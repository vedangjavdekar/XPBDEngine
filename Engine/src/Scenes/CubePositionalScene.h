#pragma once 
#include "Engine/Scene.h"
#include "Engine/Entity.h"

namespace Scenes
{
	class CubePositionalScene final: public Engine::Scene
	{
	public:
		CubePositionalScene(const std::string& sceneName);
		virtual ~CubePositionalScene();

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

	private:
		float m_Gravity = -9.8f;
		bool m_EnableGravity = true;
	};
}