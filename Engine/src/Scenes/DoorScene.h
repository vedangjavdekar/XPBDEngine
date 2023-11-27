#pragma once
#include "Engine/Scene.h"
#include "Engine/Entity.h"

namespace Scenes
{
	class DoorScene final : public Engine::Scene
	{
	public:
		DoorScene(const std::string &sceneName);
		virtual ~DoorScene();

	protected:
		void OnInit() override;

		void OnUpdate(const float substepTime) override;

		void OnStartSimulationFrame() override;

		void OnUpdatePosition(const float substepTime) override;

		void OnSolveConstraints(const float substepTime) override;

		void OnPostSolveConstraints(const float substepTime) override;

		void OnEndSimulationFrame() override;

		void OnDraw() override;

		void OnDrawEditor() override;

		void OnShutdown() override;

	private:
		void SetupEntities();
		void SetupConstraints();
		void SetupInputs();
	};
}