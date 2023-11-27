project "rlImGui"
	kind "StaticLib"
	language "C++"
    cppdialect "C++17"
    staticruntime "on"
	
    location(projectLocation)
    targetdir(targetPath)
    objdir(objectPath)

    dependson{
        "raylib",
        "imgui"
    }

    links{
        "raylib",
        "imgui"
    }

	includedirs {
        "rlImGui", 
        "%{IncludeDirs.raylib}",
        "%{IncludeDirs.raylib}/external/glfw/include",
        "%{IncludeDirs.imgui}" 
    }

	vpaths 
	{
		["Header Files"] = { "*.h"},
		["Source Files"] = {"*.cpp"},
	}

	files {"*.cpp","*.h","extras/*.h"}

	defines {
        --"IMGUI_DISABLE_OBSOLETE_FUNCTIONS",
        --"IMGUI_DISABLE_OBSOLETE_KEYIO"
        --"PLATFORM_DESKTOP"
    }

	filter "configurations:Debug"
		defines { "DEBUG" }
		symbols "On"
		
	filter "configurations:Release"
		defines { "NDEBUG" }
		optimize "On"	