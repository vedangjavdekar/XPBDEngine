#include "SceneManager.h"
#include "Scene.h"

namespace Engine
{
	void SceneManager::SwitchToScene(const std::string& sceneName)
	{
		if (m_SceneRegistry.find(sceneName) == m_SceneRegistry.end())
		{
			return;
		}

		if (m_CurrentScene)
		{
			m_CurrentScene->Shutdown();
			m_CurrentScene = nullptr;
		}

		m_CurrentScene = m_SceneRegistry.at(sceneName);
		m_CurrentScene->Init();
	}

	void SceneManager::HandleWindowResize()
	{
		for (auto& [sceneName, scene] : m_SceneRegistry)
		{
			scene->HandleWindowResize();
		}
	}

	std::shared_ptr<Scene> SceneManager::CurrentScene() const
	{
		return m_CurrentScene;
	}

	const std::vector<std::string>& SceneManager::GetSceneNames() const
	{
		return m_SceneNames;
	}

	void SceneManager::UnloadAll()
	{	
		m_CurrentScene->Shutdown();
		m_CurrentScene = nullptr;

		m_SceneNames.clear();
		m_SceneRegistry.clear();
	}

	bool SceneManager::HasValidScene() const
	{
		return (m_CurrentScene.get());
	}
}