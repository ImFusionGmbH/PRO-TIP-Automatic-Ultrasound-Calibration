#pragma once

#include <ImFusion/Base/AlgorithmControllerFactory.h>
#include <ImFusion/Base/AlgorithmFactory.h>
#include <ImFusion/Base/ImFusionPlugin.h>

namespace ImFusion
{
	class Algorithm;
	class AlgorithmController;

	class ConeCalibrationAlgorithmFactory : public AlgorithmFactory
	{
	public:
		ConeCalibrationAlgorithmFactory();
	};

	class ConeCalibrationControllerFactory : public AlgorithmControllerFactory
	{
	public:
		AlgorithmController* create(Algorithm* a) const override;
	};

	class ConeCalibrationPlugin : public ImFusionPlugin
	{
	public:
		ConeCalibrationPlugin();
		~ConeCalibrationPlugin() override;

		const AlgorithmFactory* getAlgorithmFactory() override;

		const AlgorithmControllerFactory* getAlgorithmControllerFactory() override;

	private:
		AlgorithmFactory* m_algFactory;
		AlgorithmControllerFactory* m_algCtrlFactory;
	};

}
