#include "ConeCalibrationPlugin.h"

#include "ConeCalibrationAlgorithm.h"

#include <ImFusion/Core/Filesystem/Directory.h>
#include <ImFusion/Core/Resource/Resource.h>
#include <ImFusion/GUI/DefaultAlgorithmController.h>


#ifdef WIN32
extern "C" __declspec(dllexport) ImFusion::ImFusionPlugin* createPlugin()
#else
extern "C" ImFusion::ImFusionPlugin* createPlugin()
#endif
{
	return new ImFusion::ConeCalibrationPlugin;
}


namespace ImFusion
{
	ConeCalibrationAlgorithmFactory::ConeCalibrationAlgorithmFactory()
	{
		registerAlgorithm<ConeCalibrationAlgorithm>("Ultrasound;Cone Calibration (github)");
	}

	AlgorithmController* ConeCalibrationControllerFactory::create(Algorithm* a) const
	{
		if (auto alg = dynamic_cast<ConeCalibrationAlgorithm*>(a))
			return new DefaultAlgorithmController(alg, "", true);

		return nullptr;
	}

	ConeCalibrationPlugin::ConeCalibrationPlugin()
	{
		m_algFactory = new ConeCalibrationAlgorithmFactory;
		m_algCtrlFactory = new ConeCalibrationControllerFactory;
	}

	ConeCalibrationPlugin::~ConeCalibrationPlugin() = default;


	const ImFusion::AlgorithmFactory* ConeCalibrationPlugin::getAlgorithmFactory() { return m_algFactory; }


	const ImFusion::AlgorithmControllerFactory* ConeCalibrationPlugin::getAlgorithmControllerFactory() { return m_algCtrlFactory; }

}
