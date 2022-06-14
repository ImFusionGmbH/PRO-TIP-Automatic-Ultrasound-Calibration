#include "Common.h"

#include <ImFusion/Base/Algorithm.h>
#include <ImFusion/Base/Geometry.h>
#include <ImFusion/Core/Parameter.h>

#include <unordered_map>

namespace ImFusion
{
	class SharedImageSet;
	class UltrasoundSweep;
	class UltrasoundCalibration;

	/**
	  * Algorithm that automatically calibrates two or more ultrasound sweeps.
	  * The sweeps must be taken with a "Cone Phantom" as described in the paper "PRO-TIP: Phantom for RObust automatic
	  * ultrasound calibration by TIP detection" (Ronchetti et al. 2022)
	  * The algorithm detects and matches cone tips between the sweeps and uses these matches to estimate the calibration.
	  * The calibration is automatically refined using image based optimization.
	  */
	class ConeCalibrationAlgorithm : public Algorithm
	{
	public:
		class TrackedCone
		{
		public:
			TrackedCone(int sweep, int frame, double height, const Geometry::Rectangle& box, const vec3& tip);

			double scoreMatch(int frame, const Geometry::Rectangle& box);
			void addFrame(int frame, double height, const Geometry::Rectangle& box, const vec3& tip);

			int sweepIdx;
			int lastFrame;

			std::vector<int> frames;
			std::vector<double> heights;
			std::vector<vec3> tipPositions;

			Geometry::Rectangle lastBox;
		};

		struct Tip
		{
			int sweepIndex;
			int frame;
			vec3 position;
			double height;
			mat4 tracking;
		};

		struct EstimatedCalibration
		{
			mat4 calibration;
			Eigen::Matrix<double, 9, 1> calibrationParams;

			double score;

			double imageScore;
		};

		struct Matches
		{
			size_t size() const { return rows.size(); }
			void add(const ConeCalibrationAlgorithm::Tip& tipA, const ConeCalibrationAlgorithm::Tip& tipB);

			std::vector<Eigen::Matrix<double, 3, 9>> rows;
			std::vector<vec3> targets;
			std::vector<ConeCalibrationAlgorithm::Tip> tipsA;
			std::vector<ConeCalibrationAlgorithm::Tip> tipsB;
		};

		explicit ConeCalibrationAlgorithm(std::vector<UltrasoundSweep*> sweeps);
		~ConeCalibrationAlgorithm() override;

		static bool createCompatible(const DataList& data, Algorithm** a = nullptr);

		void compute() override;

		int detectTips(int sweepIndex);
		std::vector<TrackedCone> detectCones(SharedImageSet* segmentation, int sweepIndex);

		std::vector<Tip> m_tips;
		std::vector<TrackedCone> m_cones;
		std::vector<EstimatedCalibration> m_calibrations;
		Matches m_matches;

		Parameter<std::string> p_modelPath = {"modelPath", "cones.yaml", *this};

		Parameter<double> p_heightEps = {"heightEps", 3.0, *this};
		Parameter<int> p_imageFrames = {"imageFrames", 30, *this};
		Parameter<int> p_imageBasedRepeats = {"imageBasedRepeats", 2, *this};

		Parameter<int> p_smoothing = {"smoothing", 5, *this};
		Parameter<int> p_minFrames = {"minFrames", 10, *this};

		Parameter<double> p_eps = {"eps", 10.0, *this};
		Parameter<int> p_ransacIterations = {"ransacIterations", 50000, *this};
		Parameter<int> p_ransacKeepBest = {"ransacKeepBest", 5, *this};

		Parameter<double> p_optimizationThreshold = {"optimizationThreshold", 10, *this};

	private:
		std::unique_ptr<UltrasoundCalibration>
			createCalibrationAlgorithm(const std::vector<UltrasoundSweep*>& sweeps, int mode, bool optimizeTime, Progress* progress) const;
		std::vector<EstimatedCalibration> runRANSAC(const Matches& matches);

		std::vector<UltrasoundSweep*> m_sweeps;    ///< All ultrasound sweeps to be used
		std::vector<std::unique_ptr<SharedImageSet>> m_segmentations;

		std::string m_previousModelPath;
		std::vector<std::vector<TrackedCone>> m_allCones;
	};
}