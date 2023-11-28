#include "Application.h"

#include <iostream>

#include "Scene.h"

#include "raylib.h"
#include "raymath.h"

#include "imgui.h"
#include "imgui_internal.h"
#include "rlImGui.h"

// Scenes
#include "Scenes/SceneNames.h"
#include "Scenes/ParticlesScene.h"
#include "Scenes/CubePositionalScene.h"
#include "Scenes/CubeRotationalScene.h"
#include "Scenes/CubeHingeScene.h"
#include "Scenes/DoorScene.h"

#include "Utils/PathUtils.h"

namespace Engine
{
	namespace Constants
	{
		const size_t DOCKSPACE_ID = 256;
	}

	Application *Application::s_Instance = nullptr;

	Application::Application(ApplicationProps props)
		: m_ApplicationProps(props),
		  m_IsRunning(false),
		  m_SimualtionControls()
	{
		s_Instance = this;
	}

	Application::~Application()
	{
		s_Instance = nullptr;
	}

	void Application::InitApplicationResources()
	{
		/**
		 * Reference: https://github.com/raysan5/raylib/blob/master/examples/shaders/shaders_basic_lighting.c
		 */
		// Load plane model from a generated mesh
		m_ApplicationResources.GroundPlane = LoadModelFromMesh(GenMeshPlane(10.0f, 10.0f, 3, 3));
		m_ApplicationResources.CubeModel = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));

		// Load basic lighting shader
		m_ApplicationResources.LightingShader = LoadShader(
			TextFormat("%s\\shaders\\lighting.vs", Utils::Path::GetDataPath().c_str()),
			TextFormat("%s\\shaders\\lighting.fs", Utils::Path::GetDataPath().c_str()));

		// Get some required shader locations
		m_ApplicationResources.LightingShader.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(m_ApplicationResources.LightingShader, "viewPos");
		// NOTE: "matModel" location name is automatically assigned on shader loading,
		// no need to get the location again if using that uniform name
		// shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocation(shader, "matModel");

		// Ambient light level (some basic lighting)
		int ambientLoc = GetShaderLocation(m_ApplicationResources.LightingShader, "ambient");
		float ambientVal[4] = {0.1f, 0.1f, 0.1f, 1.0f};
		SetShaderValue(m_ApplicationResources.LightingShader, ambientLoc, ambientVal, SHADER_UNIFORM_VEC4);

		// Assign out lighting shader to model
		m_ApplicationResources.GroundPlane.materials[0].shader = m_ApplicationResources.LightingShader;
		m_ApplicationResources.CubeModel.materials[0].shader = m_ApplicationResources.LightingShader;

		m_ApplicationResources.Lights[0] = CreateLight(LIGHT_DIRECTIONAL, Vector3{-2, 1, -2}, Vector3Zero(), WHITE, m_ApplicationResources.LightingShader);
		m_ApplicationResources.Lights[1] = CreateLight(LIGHT_POINT, Vector3{0, 2, 3}, Vector3Zero(), BLUE, m_ApplicationResources.LightingShader);
	}

	void Application::CleanupApplicationResources()
	{
		UnloadShader(m_ApplicationResources.LightingShader);
		UnloadModel(m_ApplicationResources.GroundPlane);
		UnloadModel(m_ApplicationResources.CubeModel);
	}

	void Application::DrawTitleBarMenu()
	{
		if (ImGui::BeginMainMenuBar())
		{
			if (ImGui::BeginMenu("File"))
			{
				if (ImGui::MenuItem("Exit"))
				{
					Close();
				}

				ImGui::EndMenu();
			}

			if (ImGui::BeginMenu("Scene"))
			{
				const auto &menuItemNames = m_SceneManager.GetSceneNames();
				for (const auto &menuItem : menuItemNames)
				{
					if (ImGui::MenuItem(menuItem.c_str()))
					{
						m_SimualtionControls.ForceStop();
						SwitchToScene(menuItem.c_str());
					}
				}

				ImGui::EndMenu();
			}
			ImGui::EndMainMenuBar();
		}
	}

	void Application::SwitchToScene(const char *sceneName)
	{
		m_SceneManager.SwitchToScene(sceneName);
		m_SimualtionControls.AttachToScene(m_SceneManager.CurrentScene());
	}

	void Application::Run()
	{
		if (m_IsRunning)
		{
			TraceLog(LOG_FATAL, "Only one instance of the application can be running at any time.\n");
		}

		Init();

		while (!(WindowShouldClose() && m_IsRunning))
		{
			const float deltaTime = GetFrameTime();
			if (IsWindowResized())
			{
				m_SceneManager.HandleWindowResize();
			}

			Update(deltaTime);

			Draw();
		}

		rlImGuiShutdown();

		m_SceneManager.UnloadAll();
		CleanupApplicationResources();

		CloseWindow();
	}

	void Application::Close()
	{
		m_IsRunning = false;
	}

	Application &Application::Get()
	{
		return *s_Instance;
	}

	ApplicationResources &Application::GetResources()
	{
		return m_ApplicationResources;
	}

	const ApplicationResources &Application::GetResources() const
	{
		return m_ApplicationResources;
	}

	bool Application::CheckUpdateMode(SimulationControls::UpdateMode mode)
	{
		return Application::Get().m_SimualtionControls.CheckUpdateMode(mode);
	}

	void Application::Init()
	{
		SetConfigFlags(FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT | FLAG_WINDOW_RESIZABLE);
		InitWindow(m_ApplicationProps.Width, m_ApplicationProps.Height, m_ApplicationProps.Title.c_str());
		SetTargetFPS(144);
		rlImGuiSetup(true);

		ImGuiIO &io = ImGui::GetIO();
		io.ConfigWindowsMoveFromTitleBarOnly = true;
		io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
		io.ConfigDockingWithShift = false;

		InitApplicationResources();

		m_SceneManager.LoadScene<Scenes::ParticlesScene>(Scenes::ParticlesSceneName);
		m_SceneManager.LoadScene<Scenes::CubePositionalScene>(Scenes::CubePositionalSceneName);
		m_SceneManager.LoadScene<Scenes::CubeRotationalScene>(Scenes::CubeRotationalSceneName);
		m_SceneManager.LoadScene<Scenes::CubeHingeScene>(Scenes::CubeHingeSceneName);
		m_SceneManager.LoadScene<Scenes::DoorScene>(Scenes::DoorSceneName);

		m_IsRunning = true;

		SwitchToScene(Scenes::DoorSceneName);
	}

	void Application::Update(const float deltaTime)
	{
		if (!m_SceneManager.HasValidScene())
		{
			return;
		}

		m_DebugDrawing.PreRender(deltaTime);

		m_SceneManager.CurrentScene()->ProcessEvents();

		const Camera &sceneCamera = m_SceneManager.CurrentScene()->GetSceneCamera();
		// Update the shader with the camera view vector (points towards { 0.0f, 0.0f, 0.0f })
		float cameraPos[3] = {sceneCamera.position.x, sceneCamera.position.y, sceneCamera.position.z};
		SetShaderValue(m_ApplicationResources.LightingShader, m_ApplicationResources.LightingShader.locs[SHADER_LOC_VECTOR_VIEW], cameraPos, SHADER_UNIFORM_VEC3);

		m_SceneManager.CurrentScene()->Update(deltaTime);

		m_SimualtionControls.Update(deltaTime);
	}

	void Application::Draw()
	{
		BeginDrawing();
		ClearBackground(BLACK);

		if (m_SceneManager.HasValidScene())
		{
			std::shared_ptr<Scene> currentScene = m_SceneManager.CurrentScene();

			if (currentScene->ShouldRender())
			{
				currentScene->BeginScene();

				currentScene->Draw();
				m_DebugDrawing.Render();
				currentScene->EndScene();
			}
		}

		// start ImGui Conent
		rlImGuiBegin();

		ImGui::DockSpaceOverViewport(ImGui::GetMainViewport());
		// ImGui::ShowDemoWindow(nullptr);

		DrawTitleBarMenu();

		if (m_SceneManager.HasValidScene())
		{
			std::shared_ptr<Scene> currentScene = m_SceneManager.CurrentScene();

			ImGui::SetNextWindowSizeConstraints(ImVec2(800, 450), ImVec2((float)GetScreenWidth(), (float)GetScreenHeight()));

			if (ImGui::Begin("Viewport", nullptr, ImGuiWindowFlags_NoScrollbar))
			{
				ImGui::IsWindowFocused(ImGuiFocusedFlags_ChildWindows);
				rlImGuiImageRenderTextureFit(&currentScene->GetViewportTexture(), true);
			}

			ImGui::End();

			currentScene->DrawEditor();

			m_SimualtionControls.DrawControls();

			if (m_DebugDrawing.DrawSettings())
			{
				currentScene->MarkDirty();
			}
		}

		// end ImGui Content
		rlImGuiEnd();

		EndDrawing();
	}

}