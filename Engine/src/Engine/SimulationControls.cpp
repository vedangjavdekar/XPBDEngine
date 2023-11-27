#include "SimulationControls.h"
#include "Scene.h"
#include "imgui.h"
#include "rlImGui.h"

namespace Engine
{
	void SimulationControls::AttachToScene(const std::shared_ptr<Scene> &scene)
	{
		m_CurrentScene = scene;
	}

	void SimulationControls::Update(const float deltaTime)
	{
		if (!m_CurrentScene)
		{
			return;
		}

		if (m_UpdateMode > UpdateMode::PAUSED)
		{
			m_SimulationTime += deltaTime;
			m_CurrentScene->Simulate(deltaTime);
		}

		if (m_UpdateMode == UpdateMode::STEP)
		{
			m_UpdateMode = UpdateMode::PAUSED;
		}
	}

	void SimulationControls::Play()
	{
		m_UpdateMode = UpdateMode::PLAY;
	}

	void SimulationControls::Step()
	{
		m_UpdateMode = UpdateMode::STEP;
	}

	void SimulationControls::Pause()
	{
		m_UpdateMode = UpdateMode::PAUSED;
	}

	void SimulationControls::Reset()
	{
		m_UpdateMode = UpdateMode::PAUSED;
		if (m_CurrentScene)
		{
			m_CurrentScene->Init();
		}
	}

	void SimulationControls::DrawControls()
	{
		ImGui::Begin("Simulation Controls");
		if (ImGui::Button(ICON_FA_ARROW_ROTATE_LEFT))
		{
			Reset();
		}

		ImGui::SameLine();
		if ((m_UpdateMode == UpdateMode::PAUSED) && ImGui::Button(ICON_FA_PLAY))
		{
			Play();
		}
		else if ((m_UpdateMode == UpdateMode::PLAY) && ImGui::Button(ICON_FA_PAUSE))
		{
			Pause();
		}

		ImGui::SameLine();
		if (ImGui::Button(ICON_FA_FORWARD_STEP))
		{
			Step();
		}

		ImGui::End();
	}

	bool SimulationControls::CheckUpdateMode(UpdateMode mode) const
	{
		return m_UpdateMode == mode;
	}

	void SimulationControls::ForceStop()
	{
		Pause();
		Reset();
	}
}