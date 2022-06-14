#include "ConeCalibrationAlgorithm.h"

#include <ImFusion/Base/ConnectivityAnalysis.h>
#include <ImFusion/Base/Optimizer.h>
#include <ImFusion/Base/PoseProcessing.h>
#include <ImFusion/Base/RANSAC.h>
#include <ImFusion/Base/SharedImageSet.h>
#include <ImFusion/Base/TrackingStream.h>
#include <ImFusion/Base/Utils/DataLogger.h>
#include <ImFusion/Core/Log.h>
#include <ImFusion/Core/Timer.h>
#include <ImFusion/Ext/fmt/format.h>
#include <ImFusion/ML/MachineLearningModel.h>
#include <ImFusion/ML/Operations.h>
#include <ImFusion/Seg/SegmentedStructure.h>
#include <ImFusion/US/UltrasoundCalibration.h>
#include <ImFusion/US/UltrasoundSweep.h>

#undef IMFUSION_LOG_DEFAULT_CATEGORY
#define IMFUSION_LOG_DEFAULT_CATEGORY "ConeCalibrationAlgorithm"


namespace ImFusion
{
	namespace
	{
		struct ConnectedComponentStats
		{
			int area = 0;
			vec3i bbMinPix = vec3i(std::numeric_limits<int>::max(), std::numeric_limits<int>::max(), 0);
			vec3i bbMaxPix = vec3i(0, 0, 0);
		};

		std::unique_ptr<TypedImage<unsigned short>>
			computeConnectedComponentsWithStats(const TypedImage<unsigned char>* image,
												std::vector<ConnectedComponentStats>& ccStats,
												ConnectivityAnalysis::Connectivity = ConnectivityAnalysis::CONNECT8_2D,
												unsigned char threshold = 1)
		{
			vec3i imageSize = vec3i(image->width(), image->height(), 1);

			auto connectedComponents = TypedImage<unsigned short>::create(imageSize, 1);
			Timer timer;
			int numLabels = ConnectivityAnalysis::computeConnectedComponents(
				image, connectedComponents.get(), ConnectivityAnalysis::CONNECT8_2D, -1, -1, threshold);

			ccStats.resize(numLabels + 1);

			const unsigned short* data = connectedComponents->pointer();
			for (int i = 0; i < imageSize[1]; i++)
			{
				for (int j = 0; j < imageSize[0]; j++)
				{
					auto label = data[i * imageSize[0] + j];
					ccStats[label].area += 1;
					ccStats[label].bbMinPix = vec3i(j, i, 0).cwiseMin(ccStats[label].bbMinPix);
					ccStats[label].bbMaxPix = vec3i(j, i, 0).cwiseMax(ccStats[label].bbMaxPix);
				}
			}

			return connectedComponents;
		}

		template <class T>
		std::vector<T> convolution(const std::vector<T>& x, const std::vector<double>& filter)
		{
			std::vector<T> res;
			for (int i = 0; i < x.size() - filter.size() + 1; i++)
			{
				T tmp = 0;
				for (int j = 0; j < filter.size(); j++)
					tmp += filter[j] * x[i + j];
				res.push_back(tmp);
			}
			return res;
		}
	}

	ConeCalibrationAlgorithm::TrackedCone::TrackedCone(int sweep, int frame, double height, const Geometry::Rectangle& box, const vec3& tip)
		: lastBox(box)
		, lastFrame(frame)
		, sweepIdx(sweep)
	{
		frames.push_back(frame);
		heights.push_back(height);
		tipPositions.push_back(tip);
	}

	double ConeCalibrationAlgorithm::TrackedCone::scoreMatch(int frame, const Geometry::Rectangle& box)
	{
		if (lastFrame == frame || frame >= lastFrame + 3)
			return 0.0;

		auto intersection = lastBox.intersected(box);
		return (intersection.width() * intersection.height()) / (box.width() * box.height());
	}


	void ConeCalibrationAlgorithm::TrackedCone::addFrame(int frame, double height, const Geometry::Rectangle& box, const vec3& tip)
	{
		frames.push_back(frame);
		heights.push_back(height);
		lastBox = box;
		lastFrame = frame;
		tipPositions.push_back(tip);
	}

	void ConeCalibrationAlgorithm::Matches::add(const ConeCalibrationAlgorithm::Tip& tipA, const ConeCalibrationAlgorithm::Tip& tipB)
	{
		tipsA.push_back(tipA);
		tipsB.push_back(tipB);

		mat3 Ra = tipA.tracking.block<3, 3>(0, 0);
		mat3 Rb = tipB.tracking.block<3, 3>(0, 0);
		vec3 ta = tipA.tracking.block<3, 1>(0, 3);
		vec3 tb = tipB.tracking.block<3, 1>(0, 3);
		Eigen::Matrix<double, 3, 9> M = Eigen::Matrix<double, 3, 9>::Zero();

		M.block<3, 3>(0, 0) = Ra * tipA.position[0] - Rb * tipB.position[0];
		M.block<3, 3>(0, 3) = Ra * tipA.position[1] - Rb * tipB.position[1];
		M.block<3, 3>(0, 6) = Ra - Rb;
		rows.push_back(M);
		targets.emplace_back(tb - ta);
	}

	ConeCalibrationAlgorithm::ConeCalibrationAlgorithm(std::vector<UltrasoundSweep*> sweeps)
		: m_sweeps(std::move(sweeps))
	{
		p_heightEps.setLabel("Max height difference");
		p_heightEps.setAttribute("suffix", "mm");
		p_imageFrames.setLabel("Number of frames for refinement");
		p_imageBasedRepeats.setLabel("Refinement iterations");

		p_modelPath.setLabel("Model");
		p_modelPath.setType(Properties::ParamType::Path);
		p_minFrames.setLabel("Minimum detections");
		p_eps.setLabel("RANSAC epsilon");
		p_eps.setAttribute("suffix", "mm");
		p_ransacIterations.setLabel("RANSAC iterations");
		p_ransacKeepBest.setLabel("RANSAC keep best candidates");
		p_optimizationThreshold.setLabel("RANSAC early discard threshold");
		p_optimizationThreshold.setAttribute("suffix", "mm");
	}

	ConeCalibrationAlgorithm::~ConeCalibrationAlgorithm() = default;

	bool ConeCalibrationAlgorithm::createCompatible(const DataList& data, Algorithm** a)
	{
		if (data.size() < 2)
			return false;
		auto img = data.getImages(Data::IMAGESET, Data::ULTRASOUND);
		if (img.size() != data.size())
			return false;
		std::vector<UltrasoundSweep*> sweeps(img.size(), nullptr);
		for (int i = 0; i < data.size(); i++)
		{
			if (auto us = dynamic_cast<UltrasoundSweep*>(img[i]))
				sweeps[i] = us;
			else
				return false;
		}
		if (a)
			*a = new ConeCalibrationAlgorithm(sweeps);

		return true;
	}

	void ConeCalibrationAlgorithm::compute()
	{
		m_status = Error;

		if (m_sweeps.empty())
			return;

		// If the model changed, clear segmentations so that they will be recomputed
		if (m_previousModelPath != p_modelPath.value())
		{
			m_segmentations.clear();
			m_allCones.clear();
			m_previousModelPath = p_modelPath.value();
		}

		m_cones.clear();
		m_tips.clear();

		Progress::Task mainTask(m_progress, static_cast<int>(m_sweeps.size() + p_imageBasedRepeats.value() + 1), "Cone calibration");

		for (int i = 0; i < m_sweeps.size(); i++)
		{
			detectTips(i);
			mainTask.incrementStep();
		}

		// Match tips based on height difference
		Matches matches;
		for (int i = 0; i < m_tips.size(); i++)
		{
			const Tip& tipA = m_tips[i];
			for (int j = i + 1; j < m_tips.size(); j++)
			{
				const Tip& tipB = m_tips[j];
				if (tipA.sweepIndex != tipB.sweepIndex)
				{
					if (std::abs(tipA.height - tipB.height) <= p_heightEps)
					{
						LOG_INFO("Matched cones with heights: " << tipA.height << " " << tipB.height);
						matches.add(tipA, tipB);
					}
				}
			}
		}
		LOG_INFO("Got " << matches.targets.size() << " matches");
		m_matches = matches;

		if (matches.targets.size() < 4)
		{
			LOG_ERROR("Insufficient number of matches");
			for (int i = 0; i < m_segmentations.size(); i++)
				DataLogger::addImage(m_segmentations[i]->clone2().release(), fmt::format("{} Segmentation", m_sweeps[i]->name()));

			return;
		}

		std::vector<EstimatedCalibration> ransacResults = runRANSAC(matches);
		mainTask.incrementStep();

		if (ransacResults.empty())
		{
			LOG_ERROR("RANSAC didn't find any calibration");
			for (int i = 0; i < m_segmentations.size(); i++)
				DataLogger::addImage(m_segmentations[i]->clone2().release(), fmt::format("{} Segmentation", m_sweeps[i]->name()));

			return;
		}

		m_calibrations.clear();

		auto imageBasedCalibration = createCalibrationAlgorithm(m_sweeps, 2, true, m_progress);

		double bestImageScore = 0;
		mat4 bestImageCal;
		mat4 bestSegCal;

		for (auto& calib : ransacResults)
		{
			LOG_DEBUG("Candidate calibration " << calib.calibration);

			for (auto* sweep : m_sweeps)
				sweep->tracking()->setCalibration(calib.calibration);
			imageBasedCalibration->defineReferenceState();
			calib.imageScore = imageBasedCalibration->evaluate(0, nullptr, nullptr);

			LOG_DEBUG("Image score: " << calib.imageScore);

			if (calib.imageScore > bestImageScore)
			{
				bestImageScore = calib.imageScore;
				bestImageCal = calib.calibration;
			}

			m_calibrations.push_back(std::move(calib));
		}

		LOG_INFO("Refining using image content");
		for (auto* sweep : m_sweeps)
			sweep->tracking()->setCalibration(bestImageCal);

		imageBasedCalibration->centerSweeps();
		for (int i = 0; i < p_imageBasedRepeats; i++)
		{
			imageBasedCalibration->compute();
			mainTask.incrementStep();
		}

		for (auto& segmentation : m_segmentations)
			for (int i = 0; i < segmentation->size(); i++)
				segmentation->setMatrix(m_sweeps[0]->matrix(i), i);

		for (int i = 0; i < m_segmentations.size(); i++)
			DataLogger::addImage(m_segmentations[i]->clone2().release(), fmt::format("{} Segmentation", m_sweeps[i]->name()));

		m_status = Success;
	}


	int ConeCalibrationAlgorithm::detectTips(int sweepIndex)
	{
		UltrasoundSweep* us = m_sweeps[sweepIndex];

		Timer timer;
		if (sweepIndex >= m_segmentations.size())
		{
			ML::MachineLearningModel model(p_modelPath);
			model.setProgress(m_progress);
			std::unique_ptr<SharedImageSet> segmentation = model.predict(*us);
			if (!segmentation)
			{
				LOG_ERROR("Could not run segmentation");
				return 0;
			}
			LOG_INFO("Model inference: " << timer.formatPassedAndReset());
			auto cones = detectCones(segmentation.get(), sweepIndex);
			m_segmentations.push_back(std::move(segmentation));
			m_allCones.push_back(std::move(cones));
		}

		int tipCount = 0;
		int smoothing = std::max(1, p_smoothing.value());
		std::vector<double> smoothingFilter(smoothing, 1.0 / double(smoothing));

		auto frameGeometry = us->frameGeometry(US::CoordinateSystem::Image);

		for (auto& cone : m_allCones[sweepIndex])
		{
			if (cone.heights.size() >= p_minFrames)
			{
				auto heights = convolution(cone.heights, smoothingFilter);

				double maxHeight = 0;
				int tipIndex = 0;
				for (int i = 0; i < heights.size(); i++)
				{
					if (heights[i] >= maxHeight)
					{
						maxHeight = heights[i];
						tipIndex = i;
					}
				}
				if (maxHeight >= 15)
				{
					int frame = cone.frames[tipIndex + (smoothing - 1) / 2];
					vec3 position = cone.tipPositions[tipIndex + (smoothing - 1) / 2];
					position = m_segmentations[sweepIndex]->img(frame)->pixelToImage(position);

					if (!frameGeometry->contains({position[0], position[1]}))
					{
						LOG_WARN("Skipped cone because tip is outside of frame geometry");
						continue;
					}

					m_tips.push_back({sweepIndex, frame, position, maxHeight, us->tracking()->rawMatrix(us->timestamp(frame))});
					tipCount++;
					LOG_INFO("Found tip at position " << position.transpose() << ", height " << maxHeight << " on frame " << frame
													  << "(frames: " << cone.heights.size() << " " << sweepIndex << ")");
					m_cones.push_back(cone);
				}
				else
				{
					LOG_WARN("Skipped cone because height is too low (" << maxHeight << ")");
				}
			}
			else
			{
				LOG_WARN("Skipped cone because it had only " << cone.heights.size() << " detections");
			}
		}

		return tipCount;
	}

	std::vector<ConeCalibrationAlgorithm::TrackedCone> ConeCalibrationAlgorithm::detectCones(SharedImageSet* segmentation, int sweepIndex)
	{
		Timer timer;
		std::vector<Segmentation::Line2DReferenceShape*> bases;

		// Segmentation has a spacing of 1mm, phantom is 150mm
		int basePointsThreshold = std::min(segmentation->img()->width(), 150);

		for (int frame = 0; frame < segmentation->size(); frame++)
		{
			// find base of the phantom
			std::vector<vec2> points;

			MemImage* seg = segmentation->mem(frame);
			unsigned char* data = seg->typed<unsigned char>()->pointer();
			for (int i = 0; i < seg->height(); i++)
			{
				for (int j = 0; j < seg->width(); j++)
				{
					if (data[i * seg->width() + j] == 1)
						points.emplace_back(j, i);
				}
			}

			if (points.size() < basePointsThreshold)
			{
				LOG_DEBUG("Skipping frame " << frame << " because base was not found" << points.size());
				bases.push_back(nullptr);
				continue;
			}

			// Use ransac to fit a line and find inliers, skip the frame if there are not enough inliers
			std::vector<vec2> basePoints;
			vec3 tmp;
			Geometry::fitLineRANSAC(points, tmp, basePoints, 2.0);

			if (basePoints.size() < basePointsThreshold)
			{
				LOG_DEBUG("Skipping frame " << frame << " because base was not found");
				bases.push_back(nullptr);
				continue;
			}

			bases.push_back(new Segmentation::Line2DReferenceShape(basePoints));
			const Segmentation::Line2DReferenceShape* base = bases.back();

			// mask pixels under the base
			for (int i = 0; i < seg->height(); i++)
			{
				for (int j = 0; j < seg->width(); j++)
				{
					if (base->worldToLocal(vec3(double(j), double(i), 0.0))[1] > 2.0)
						data[i * seg->width() + j] = 0;
				}
			}
			segmentation->get(frame)->setDirtyMem();
		}

		// Remove base points, just keep cone segmentations
		auto coneFrames = ML::ReplaceLabelsValuesOperation({1}, {0}).processImages(segmentation->clone2());

		std::vector<TrackedCone> cones;
		for (int frame = 0; frame < segmentation->size(); frame++)
		{
			const Segmentation::Line2DReferenceShape* base = bases[frame];
			if (base == nullptr)
				continue;

			std::vector<ConnectedComponentStats> ccStats;
			auto CCs = computeConnectedComponentsWithStats(coneFrames->mem(frame)->typed<unsigned char>(), ccStats);
			auto* ccData = CCs->pointer();

			// skip first because it is the background
			for (int c = 1; c < ccStats.size(); c++)
			{
				ConnectedComponentStats& cc = ccStats[c];

				int sx = cc.bbMinPix[0];
				int sy = cc.bbMinPix[1];
				int ex = cc.bbMaxPix[0];
				int ey = cc.bbMaxPix[1];
				Geometry::Rectangle bbox({sx, sy}, {ex, ey});
				double bestScore = 0.0;
				int bestIndex = -1;
				for (int i = 0; i < cones.size(); i++)
				{
					double score = cones[i].scoreMatch(frame, bbox);
					if (score > bestScore)
					{
						bestScore = score;
						bestIndex = i;
					}
				}

				vec3 tip;
				double height = 0;
				unsigned char ccLabel = 0;
				if (bestIndex > -1)
					ccLabel = bestIndex + 2;
				for (int i = sy; i <= ey; i++)
				{
					for (int j = sx; j <= ex; j++)
					{
						if (ccData[i * coneFrames->img()->width() + j] == c)
						{
							segmentation->mem(frame)->typed<unsigned char>()->setValue(ccLabel, j, i);
							vec3 p = vec3(double(j), double(i), 0.0);
							vec3 q = base->worldToLocal(p);
							double h = -q[1];
							if (h > height)
							{
								height = h;
								tip = p;
							}
						}
					}
				}
				if (cc.area < 25)
					continue;

				if (bestIndex > -1)
				{
					cones[bestIndex].addFrame(frame, height, bbox, tip);
				}
				else
				{
					cones.emplace_back(sweepIndex, frame, height, bbox, tip);
				}
			}
		}
		LOG_INFO("Segmentation processing: " << timer.formatPassedAndReset());
		LOG_INFO("Detected " << cones.size() << " cones");

		return cones;
	}

	std::unique_ptr<UltrasoundCalibration> ConeCalibrationAlgorithm::createCalibrationAlgorithm(const std::vector<UltrasoundSweep*>& sweeps,
																								int mode,
																								bool optimizeTime,
																								Progress* progress) const
	{
		if (sweeps.empty())
			return nullptr;

		auto imageBasedAlgorithm = std::make_unique<UltrasoundCalibration>(sweeps);
		imageBasedAlgorithm->setProgress(progress);
		std::shared_ptr<Optimizer> opt = imageBasedAlgorithm->createDefaultOptimizer();
		imageBasedAlgorithm->setOptimizer(opt);
		imageBasedAlgorithm->centerSweeps();
		imageBasedAlgorithm->setMaxFrames(p_imageFrames);
		imageBasedAlgorithm->setSimilarityMeasure(mode);
		opt->setLogging(0, 2);
		std::vector<bool> sel = opt->selection();
		if (sel.size() > 6)
			sel[6] = optimizeTime;    // enable temporal optimization
		opt->setSelection(sel);
		return imageBasedAlgorithm;
	}

	std::vector<ConeCalibrationAlgorithm::EstimatedCalibration> ConeCalibrationAlgorithm::runRANSAC(const ConeCalibrationAlgorithm::Matches& matches)
	{
		auto computeCalibration = [this](const Matches& dataset, const std::vector<size_t>& indices, EstimatedCalibration& calibOut) {
			// fill linear system
			Eigen::Matrix<double, Eigen::Dynamic, 9> A(indices.size() * 3, 9);
			Eigen::Matrix<double, Eigen::Dynamic, 1> b(indices.size() * 3);

			for (int i = 0; i < indices.size(); i++)
			{
				A.block<3, 9>(3 * i, 0) = dataset.rows[indices[i]];
				b.block<3, 1>(3 * i, 0) = dataset.targets[indices[i]];
			}

			// solve linear system
			auto sol = A.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV).setThreshold(0.0002).solve(b).eval();

			// determine rotational part
			vec3 v = sol.block<3, 1>(0, 0);
			vec3 w = sol.block<3, 1>(3, 0);
			vec3 n = v.cross(w).normalized();

			mat3 R;
			R << v, w, n;
			auto svd = R.bdcSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
			R = svd.matrixU() * svd.matrixV().transpose();

			Eigen::Matrix<double, 9, 1> tmp;
			tmp.block<3, 1>(0, 0) = R.col(0);
			tmp.block<3, 1>(3, 0) = R.col(1);

			// determine translational part
			auto C = A.block(0, 6, int(3 * indices.size()), 3);
			auto c = b - A.block(0, 0, int(3 * indices.size()), 6) * tmp.block<6, 1>(0, 0);
			// Use threshold to avoid setting a very large translation in badly conditioned problems
			tmp.block<3, 1>(6, 0) = C.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).setThreshold(0.01).solve(c).eval();

			// Fill out guess
			mat4 X = mat4::Identity();
			X.block<3, 3>(0, 0) = R;
			X.block<3, 1>(0, 3) = tmp.block<3, 1>(6, 0);

			std::vector<ConeCalibrationAlgorithm::Tip> tipsA;
			std::vector<ConeCalibrationAlgorithm::Tip> tipsB;
			for (size_t index : indices)
			{
				tipsA.push_back(dataset.tipsA[index]);
				tipsB.push_back(dataset.tipsB[index]);
			}

			calibOut.calibration = X;
			calibOut.calibrationParams.block<3, 1>(0, 0) = X.block<3, 1>(0, 0);
			calibOut.calibrationParams.block<3, 1>(3, 0) = X.block<3, 1>(0, 1);
			calibOut.calibrationParams.block<3, 1>(6, 0) = X.block<3, 1>(0, 3);

			double error = (A * calibOut.calibrationParams - b).norm();
			if (error > p_optimizationThreshold)
				return false;

			return true;
		};

		auto scoreCalibration = [](const Matches& dataset, size_t index, const EstimatedCalibration& calibration, double eps) {
			vec3 pred = dataset.rows[index] * calibration.calibrationParams;
			double error = (pred - dataset.targets[index]).norm();
			return error <= eps;
		};

		Timer timer;

		RANSAC<Matches, EstimatedCalibration> ransac(matches, computeCalibration, scoreCalibration);
		ransac.setEps(p_eps);
		ransac.setMaxIterations(p_ransacIterations);
		ransac.compute(4, p_ransacKeepBest);

		if (!ransac.results().empty())
			ransac.refine();

		LOG_INFO("Ransac took " << timer.formatPassedAndReset());

		std::vector<EstimatedCalibration> res;
		for (auto& x : ransac.results())
		{
			auto& calib = x.hypothesis;
			calib.score = x.totalScore;
			res.push_back(calib);
		}

		return res;
	}
}