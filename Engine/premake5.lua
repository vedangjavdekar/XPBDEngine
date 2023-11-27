project "Engine"
    kind "ConsoleApp"
    language "C++"
    cppdialect "C++17"
    staticruntime "on"

    targetdir(targetPath)
    objdir(objectPath)
    location(projectLocation)

    dependson {
       "raylib",
       "imgui",
       "rlImGui"
    }

    links{
       "raylib",
       "imgui",
       "rlImGui"
    }

    files { 
        "src/**.h",
        "src/**.cpp",
    }

    includedirs {
        "src",
        "%{IncludeDirs.imgui}",
        "%{IncludeDirs.raylib}",
        "%{IncludeDirs.rlImGui}",
        "%{IncludeDirs.eigen}",
        "%{IncludeDirs.eigen}/Eigen/",
    }

    defines{
        "PLATFORM_DESKTOP", 
        "GRAPHICS_API_OPENGL_43"
    }

    postbuildcommands{
        "{RMDIR} \""..targetPath.."/data\"",
        "{COPYDIR} \"%{wks.location}/data\" \"".. targetPath .. "/data\"" 
    }

    filter "configurations:Debug"
        defines { "BUILD_DEBUG" }
        symbols "On"

    filter "configurations:Release"
        defines { "BUILD_RELEASE" }
        optimize "On"