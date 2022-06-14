#pragma once

#if defined(_MSC_VER)

#	if defined(IMFUSION_TIP_DLL)
#		define IMFUSION_TIP_API __declspec(dllexport)
#	elif defined(IMFUSIONLIB_STATIC)
#		define IMFUSION_TIP_API
#	else
#		define IMFUSION_TIP_API __declspec(dllimport)
#	endif

#else

#	if defined(IMFUSION_TIP_DLL)
#		define IMFUSION_TIP_API __attribute__((visibility("default")))
#	elif defined(IMFUSIONLIB_STATIC)
#		define IMFUSION_TIP_API
#	else
#		define IMFUSION_TIP_API __attribute__((visibility("default")))
#	endif

#endif
