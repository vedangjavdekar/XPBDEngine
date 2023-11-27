#pragma once

#define DEFINE_SCENE(SceneName) const char *##SceneName##Name = #SceneName

namespace Scenes
{
    DEFINE_SCENE(ParticlesScene);
    DEFINE_SCENE(CubePositionalScene);
    DEFINE_SCENE(CubeRotationalScene);
    DEFINE_SCENE(DoorScene);
    DEFINE_SCENE(DoorScene);
}