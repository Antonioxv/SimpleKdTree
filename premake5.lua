--[[
Date: 2022-07-18 18:20:37
LastEditors: fuchaoxin
LastEditTime: 2022-07-20 03:57:23
FilePath: \KdTree\premake5.lua
--]]

workspace "SimpleKdTree"
	architecture "x64"

	configurations 
	{
		"Debug",
		"Release",
		"Dist"
	}

	
outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

project "KdTree"
	location "KdTree"
	kind "ConsoleApp"
	language "C++"

	targetdir ("bin/" .. outputdir .. "/%{prj.name}")
	objdir ("bin-intermediate/" .. outputdir .. "/%{prj.name}")

	pchheader "kdtreepch.h"  -- Enable Using pch header file.
	pchsource "KdTree/src/kdtreepch.cpp"  -- Create pch header file (for vs, gcc does not necessarily need it).

	files 
	{
		"%{prj.name}/src/**.c",
		"%{prj.name}/src/**.h",
		"%{prj.name}/src/**.cpp",
		"%{prj.name}/src/**.hpp"
	}

	includedirs
	{
		"%{prj.name}/src",  -- so that u can include KdTree.h and kdtreepch.h like: "KdTree/c.h" and "kdtreepch.h".
		-- "%{prj.name}/vendor/opencv/include",
	}

	filter "system:windows"
		cppdialect "C++17"
		staticruntime "On"
		systemversion "latest"

		defines
		{
			"KDTREE_PLATFORM_WINDOWS",
		}

		postbuildcommands
		{
			-- ("{COPY} %{cfg.buildtarget.relpath} ../bin/" .. outputdir .. "/CornellBox")
		}

	filter "configurations:Debug"
		defines 
		{
			"KDTREE_DEBUG"
		}
		symbols "On"
	
	filter "configurations:Release"
		defines 
		{
			"KDTREE_RELEASE"
		}
		optimize "On"
	
	filter "configurations:Dist"
		defines 
		{
			"KDTREE_DIST"
		}
		optimize "On"

	-- filter {"system:windows", "configurations:Release"}
