#pragma once
#include "Engine/Scene.h"
#include "Engine/Entity.h"

namespace Scenes
{
	class CubeRotationalScene final : public Engine::Scene
	{
	public:
		CubeRotationalScene(const std::string &sceneName);
		virtual ~CubeRotationalScene();

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
		void SetupEntites();
		void SetupConstraints();
		void SetupInputs();
	};
}