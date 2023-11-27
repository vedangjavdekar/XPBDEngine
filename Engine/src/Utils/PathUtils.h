#pragma once
#include "raylib.h"
#include <string>


namespace Utils::Path
{
    std::string GetDataPath()
    {
        std::string appDir(GetApplicationDirectory());
        size_t binPos = appDir.find("bin");
        if (binPos != std::string::npos)
        {
            appDir = appDir.substr(0, binPos);
        }

        return appDir + "data";
    }
}