project(ConeCalibrationPlugin)
cmake_minimum_required(VERSION 3.2.0)
message(STATUS "> Configuring ${PROJECT_NAME}")

find_package(ImFusionLib COMPONENTS ImFusionML ImFusionUS REQUIRED)

set(Sources
		ConeCalibrationAlgorithm.cpp
		ConeCalibrationPlugin.cpp
		)

set(Headers
		Common.h
		ConeCalibrationAlgorithm.h
		ConeCalibrationPlugin.h
		)

add_library(ConeCalibrationPlugin SHARED ${Sources} ${UiSources} ${Headers})
target_link_libraries(ConeCalibrationPlugin PRIVATE ImFusionLib ImFusionML ImFusionUS)


imfusion_set_common_target_properties()
imfusion_provide_ide_instructions()

# install models as resources
imfusion_compile_resource_repository(ConeCalibrationPluginResources
		FILES
		${CMAKE_CURRENT_SOURCE_DIR}/model/cones.pt
		${CMAKE_CURRENT_SOURCE_DIR}/model/cones.yaml
		BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/model
		COMPRESS
		)
