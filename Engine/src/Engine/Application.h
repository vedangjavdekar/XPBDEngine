#pragma once
#include <string>
#include "SceneManager.h"
#include "Scene.h"
#include "SimulationControls.h"
#include "DebugDrawing.h"
#include "rlights.h"

namespace Engine
{
    struct ApplicationProps
    {
        size_t Width;
        size_t Height;
        std::string Title;
        
        uint8_t DefaultCloseKey = 27; // Escape Key
        bool FullScreen = false;
    };

    struct ApplicationResources
    {
        Light Lights[2];
        Shader LightingShader;
        Model GroundPlane;
        Model CubeModel;
    };


    class Application
    {
    public:
        Application(ApplicationProps props);
        ~Application();

        void Run();
        void Close();
        
        static Application& Get();
        
        ApplicationResources& GetResources();
        const ApplicationResources& GetResources() const;

        static bool CheckUpdateMode(SimulationControls::UpdateMode mode);
    protected:
        void Init();
        void Update(const float deltaTime);
        void Draw();

    private:
        void InitApplicationResources();
        void CleanupApplicationResources();
        void DrawTitleBarMenu();
        void SwitchToScene(const char* sceneName);
    protected:
        static Application* s_Instance;

        SceneManager m_SceneManager;
        ApplicationProps m_ApplicationProps;
        SimulationControls m_SimualtionControls;
        ApplicationResources m_ApplicationResources;
        DebugDrawing m_DebugDrawing;

        bool m_IsRunning;
    };
}