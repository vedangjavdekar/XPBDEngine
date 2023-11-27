#pragma once
#include "Engine/Scene.h"
#include "Engine/Entity.h"

#include <vector>
#include <string>
#include <memory>
#include "Eigen/Dense"

namespace Scenes
{
	class ParticlesScene final : public Engine::Scene
	{
	public:
		ParticlesScene(const std::string &sceneName);

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

		void DrawParticle(const std::string &particleName, Simulation::Entity &particle, size_t StoredForceIndex);

	private:
		float m_ParticleDrawRadius = 0.1f;
		float m_Gravity = -9.8f;
		bool m_EnableGravity = true;
		bool m_GroundCollisions = true;
	};
}