#pragma once
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

namespace Engine
{
	class Scene;

	class SceneManager
	{
	public:
		template<typename T>
		void LoadScene(const std::string& sceneName, bool switchToScene = false)
		{
			if (m_SceneRegistry.find(sceneName) == m_SceneRegistry.end())
			{
				m_SceneRegistry.emplace(sceneName, std::make_shared<T>(sceneName));
				m_SceneNames.push_back(sceneName);
			}

			if (switchToScene)
			{
				if (m_CurrentScene)
				{
					m_CurrentScene->Shutdown();
					m_CurrentScene = nullptr;
				}
				m_CurrentScene = m_SceneRegistry.at(sceneName);
				m_CurrentScene->Init();
			}
		}

		void SwitchToScene(const std::string& sceneName);

		void HandleWindowResize();

		std::shared_ptr<Scene> CurrentScene() const;

		const std::vector<std::string>& GetSceneNames() const;

		void UnloadAll();

		bool HasValidScene() const;
	private:
		std::shared_ptr<Scene> m_CurrentScene;
		std::vector<std::string> m_SceneNames;
		std::unordered_map<std::string, std::shared_ptr<Scene>> m_SceneRegistry;
	};
}