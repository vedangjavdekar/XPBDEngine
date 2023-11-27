#pragma once
#include <string>

#include "raylib.h"

namespace Engine
{
    class Scene
    {
    public:
        Scene(std::string sceneName);
        virtual ~Scene();
    
        void Init();
        void ProcessEvents();
        
        /**
        * Unconstrained Update.
        * Free from Simulation.
        * Called every frame once.
        */
        virtual void Update(const float deltaTime);

        void Simulate(const float deltaTime);
        void HandleWindowResize();

        bool ShouldRender() const;
        // Drawing Functions
        void BeginScene();
        void EndScene();
        void Draw();
        void DrawEditor();
        
        void Shutdown();

        const std::string GetName() const;
        const RenderTexture& GetViewportTexture() const;

        const int GetSubsteps() const;
        const int GetNumPosIterations() const;

        const Camera& GetSceneCamera() const;

        void MarkDirty();
    protected:
        // Setup
        virtual void OnInit() = 0;
        virtual void OnShutdown() = 0;
        
        virtual void OnUpdate(const float deltaTime) = 0;

        // Updates
        virtual void OnStartSimulationFrame() = 0;
        virtual void OnUpdatePosition(const float substepTime) = 0;
        virtual void OnSolveConstraints(const float substepTime) = 0;
        virtual void OnPostSolveConstraints(const float substepTime) = 0;
        virtual void OnEndSimulationFrame() = 0;

        
        virtual void OnDraw() = 0;
        virtual void OnDrawEditor();

    private:
        void HandleXPBDLoop(const float deltaTime);

    protected:
        bool m_IsDirty = false;

    private:
        std::string m_SceneName;
        RenderTexture m_ViewportTexture;
        Camera m_SceneCamera;

        bool m_OverrideDeltaTime = false;
        float m_DeltaTime = 0.0f;
        float m_Accumulator = 0.0f;

        int m_Substeps = 8;
        int m_NumPosIterations = 1;

        bool m_DrawGrid = false;
    };
}