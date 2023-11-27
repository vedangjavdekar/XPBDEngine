#pragma once
#include <array>
#include "raylib.h"
#include "Engine/Entity.h"
#include <Eigen/Dense>

namespace Engine
{
    constexpr const size_t MAX_DEBUG_SHAPES = 100;

    enum DebugFlags
    {
        LIGHTS,
        FORCES,
        CONSTRAINTS,
        DEBUG_FLAGS_MAX
    };

    class DebugDrawing
    {
    public:
        struct DebugLine
        {
            Vector3 From;
            Vector3 To;
            Color Color;
            float Lifetime;
        };

        struct DebugSphere
        {
            Vector3 Center;
            float Radius;
            Color Color;
            float Lifetime;
            bool IsWireframe;
        };

    public:
        static void DrawForceMarker(Color color,
            Eigen::Vector3f origin,
            Eigen::Quaternionf rotation,
            Eigen::Vector3f forcePosition,
            Eigen::Vector3f direction,
            bool isLocal = false,
            float minLength = -1.0f,
            float lifetime = -1.0f);

        static void DrawLightMarker(Color color, Eigen::Vector3f origin, float lifetime = -1.0f);
        static void DrawLightMarker(Color color, Vector3 origin, float lifetime = -1.0f);

        static void DrawDebugLine(Vector3 from, Vector3 to, Color color, float lifetime = -1.0f);

        static void DrawDebugSphere(Vector3 center, float radius, Color color, float lifetime = -1.0f, bool wireframe = false);

    protected:
        bool DrawSettings();
        void PreRender(const float deltaTime);
        void Render();

    private:
        static DebugLine RayToLine(Ray ray, Color color, float lifeTime = -1.0f);
        
        void RenderLine(const DebugLine& line);
        void RenderSphere(const DebugSphere& sphere);

        static void AddLine(DebugLine line);
        static void AddSphere(DebugSphere sphere);

        static void DeleteLine(int index);
        static void DeleteSphere(int index);
    private:
        static std::array<bool, DEBUG_FLAGS_MAX> s_Flags;

        static std::array<DebugLine, MAX_DEBUG_SHAPES> s_DebugLines;
        static int s_LinePointer;

        static std::array<DebugSphere, MAX_DEBUG_SHAPES> s_DebugSpheres;
        static int s_SpherePointer;

        static float s_ForceScale;
        static float s_MarkerScale;
        friend class Application;
    };
}
