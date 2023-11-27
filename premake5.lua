include "Dependencies.lua"

workspace "XPBDSandbox"
   configurations { "Debug", "Release" }
   architecture "x86_64"

projectLocation = "%{wks.location}/build/%{prj.name}"
basedir = "%{wks.location}/"
outputdir = "%{cfg.buildcfg}"
targetPath = basedir .. "bin/" .. outputdir
objectPath = basedir .. "bin-int/" .. outputdir 
debugPath =  basedir 

group "Core"
   include "Engine"
group ""

group "Dependencies"
   include "Engine/vendor/raylib"
   include "Engine/vendor/imgui"
   include "Engine/vendor/rlImGui"
group ""

