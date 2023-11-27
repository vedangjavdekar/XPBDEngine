#include "Scene.h"

#include "Application.h"

#include "imgui.h"
#include "rlImGui.h"
#include "raymath.h"

#include "CameraControls.h"
#include "SimulationControls.h"

namespace Engine
{
	Scene::Scene(std::string sceneName)
		:m_SceneName(sceneName), m_SceneCamera({ 0 })
	{
		m_ViewportTexture = LoadRenderTexture(GetScreenWidth(), GetScreenHeight());

		m_SceneCamera.position = Vector3{ 0.0f, 10.0f, 10.0f };
		m_SceneCamera.target = Vector3{ 0.0f, 0.0f, 0.0f };
		m_SceneCamera.up = Vector3{ 0.0f, 1.0f, 0.0f };
		m_SceneCamera.fovy = 45.0f;
		m_SceneCamera.projection = CAMERA_PERSPECTIVE;

	}

	Scene::~Scene()
	{
		UnloadRenderTexture(m_ViewportTexture);
	}

	void Scene::Init()
	{
		OnInit();
		m_IsDirty = true;
	}

	void Scene::ProcessEvents()
	{
		m_IsDirty |= Utils::Camera::UpdateCamera(&m_SceneCamera);
	}

	void Scene::Update(const float deltaTime)
	{
		OnUpdate(deltaTime);
	}

	void Scene::Simulate(const float deltaTime)
	{
		float dt = deltaTime;
		if (m_OverrideDeltaTime)
		{
			while (m_Accumulator > m_DeltaTime)
			{
				HandleXPBDLoop(m_DeltaTime);
				m_Accumulator -= m_DeltaTime;
			}
			m_Accumulator += deltaTime;
		}
		else
		{
			HandleXPBDLoop(dt);
		}
		m_IsDirty = true;
	}

	void Scene::Shutdown()
	{
		OnShutdown();
	}

	void Scene::Draw()
	{
		OnDraw();
		m_IsDirty = false;
	}

	void Scene::HandleWindowResize()
	{
		UnloadRenderTexture(m_ViewportTexture);
		m_ViewportTexture = LoadRenderTexture(GetScreenWidth(), GetScreenHeight());
		m_IsDirty = true;
	}

	bool Scene::ShouldRender() const
	{
		return m_IsDirty;
	}

	void Scene::DrawEditor()
	{
		ImGui::SetNextWindowSizeConstraints(ImVec2(300, 200), ImVec2(600, (float)GetScreenHeight()));
		ImGui::Begin(m_SceneName.c_str());

		OnDrawEditor();

		ImGui::End();
	}

	const std::string Scene::GetName() const
	{
		return m_SceneName;
	}

	const RenderTexture& Scene::GetViewportTexture() const
	{
		return m_ViewportTexture;
	}

	const int Scene::GetSubsteps() const
	{
		return m_Substeps;
	}

	const int Scene::GetNumPosIterations() const
	{
		return m_NumPosIterations;
	}

	const Camera& Scene::GetSceneCamera() const
	{
		return m_SceneCamera;
	}

	void Scene::MarkDirty()
	{
		m_IsDirty = true;
	}

	void Scene::BeginScene()
	{
		BeginTextureMode(m_ViewportTexture);
		ClearBackground(DARKGRAY);
		BeginMode3D(m_SceneCamera);

		const ApplicationResources& resources = Application::Get().GetResources();

		DrawModel(resources.GroundPlane, Vector3Zero(), 1.0f, WHITE);
		for (auto& light : resources.Lights)
		{
			if (light.type == LIGHT_POINT)
			{
				DebugDrawing::DrawLightMarker(light.color, light.position,0.0f);
			}
		}
	}

	void Scene::EndScene()
	{
		EndMode3D();
		EndTextureMode();
	}

	void Scene::OnDrawEditor()
	{
		ImGui::BeginDisabled(!Engine::Application::CheckUpdateMode(Engine::SimulationControls::UpdateMode::PAUSED));
		ImGui::Checkbox("Override Delta Time", &m_OverrideDeltaTime);
		if (m_OverrideDeltaTime)
		{
			ImGui::DragFloat("Delta Time", &m_DeltaTime, 0.01f, 0.0f, 0.033f, "%.3f", ImGuiSliderFlags_AlwaysClamp);
		}
		ImGui::EndDisabled();

		ImGui::DragInt("Substeps", &m_Substeps);
		ImGui::DragInt("PositionIterations", &m_NumPosIterations);
	}

	void Scene::HandleXPBDLoop(const float deltaTime)
	{
		OnStartSimulationFrame();

		const float subStepTime = deltaTime / (float)m_Substeps;
		for (int i = 0; i < m_Substeps; ++i)
		{
			OnUpdatePosition(subStepTime);
			OnSolveConstraints(subStepTime);
			OnPostSolveConstraints(subStepTime);
		}

		OnEndSimulationFrame();
	}
}