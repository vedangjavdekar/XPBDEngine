#pragma once
#include <memory>

namespace Engine
{
	class Scene;

	class SimulationControls
	{
	public:
		enum class UpdateMode : uint8_t
		{
			PAUSED = 0,
			STEP,
			PLAY
		};

	public:
		void AttachToScene(const std::shared_ptr<Scene> &scene);
		void Update(const float deltaTime);
		void DrawControls();

		bool CheckUpdateMode(UpdateMode mode) const;

		void ForceStop();

	private:
		void Play();
		void Pause();
		void Reset();
		void Step();

	private:
		std::shared_ptr<Scene> m_CurrentScene;
		UpdateMode m_UpdateMode;
		float m_SimulationTime;
		bool m_SimulationStarted = false;
	};
}