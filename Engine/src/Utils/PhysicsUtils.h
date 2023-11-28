#pragma once
#include "raylib.h"
#include <Eigen/Dense>
#include "Engine/Entity.h"

namespace Simulation
{
    struct Entity;
}


namespace Utils::Physics
{
    enum ActivationMode
    {
        Toggle,
        Pressed,
    };

    struct ForceInput
    {
        ActivationMode ActivationMode = ActivationMode::Pressed;
        KeyboardKey ActivationKey = KEY_NULL;
        Eigen::Vector3f ForceVector;
        Eigen::Vector3f ForcePosition;
        Simulation::Entity* Entity = nullptr;
        float MarkerScale = 0.2f;
        bool IsActive = false;
        bool SkipProcessInputsNextFrame = false;
        bool IsLocal = false;
        bool IsRotationalForce = false;

        void ProcessEvents();
        void Apply();

        void Draw();
        bool DrawSettings();
    };
}