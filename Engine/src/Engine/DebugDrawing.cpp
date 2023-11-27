#include "DebugDrawing.h"
#include "raymath.h"
#include "imgui.h"
#include "Utils/EigenToRaylib.h"
#include "Engine/Application.h"

namespace Engine
{
	static const char* EnumToName(DebugFlags flag)
	{
		constexpr const char* names[] = { "Lights", "Force", "Constraints" };
		if (flag < 0 || flag >= DEBUG_FLAGS_MAX)
		{
			return "";
		}
		return names[flag];
	}

	std::array<bool, DEBUG_FLAGS_MAX> DebugDrawing::s_Flags = { false, true, true };

	std::array<DebugDrawing::DebugLine, MAX_DEBUG_SHAPES> DebugDrawing::s_DebugLines;
	int DebugDrawing::s_LinePointer = 0;

	std::array<DebugDrawing::DebugSphere, MAX_DEBUG_SHAPES> DebugDrawing::s_DebugSpheres;
	int DebugDrawing::s_SpherePointer = 0;

	float DebugDrawing::s_ForceScale = 0.5f;
	float DebugDrawing::s_MarkerScale = 0.1f;

	bool DebugDrawing::DrawSettings()
	{
		bool result = false;
		ImGui::Begin("Debug Drawing");

		for (int i = 0; i < s_Flags.size(); ++i)
		{
			result |= ImGui::Checkbox(EnumToName((DebugFlags)i), &s_Flags[i]);
		}

		ImGui::DragFloat("Force Scale", &s_ForceScale, 0.05f, 0.05f, 1.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);
		ImGui::DragFloat("Marker Scale", &s_MarkerScale, 0.05f, 0.05f, 1.0f, "%.3f", ImGuiSliderFlags_AlwaysClamp);

		ImGui::End();

		return result;
	}

	void DebugDrawing::PreRender(const float deltaTime)
	{
		for (int i = s_LinePointer - 1; i >= 0; --i)
		{
			if (s_DebugLines[i].Lifetime >= 0.0f)
			{
				s_DebugLines[i].Lifetime -= deltaTime;
				if (s_DebugLines[i].Lifetime < 0.0f)
				{
					DeleteLine(i);
				}
			}
		}

		for (int i = s_SpherePointer - 1; i >= 0; --i)
		{
			if (s_DebugSpheres[i].Lifetime >= 0.0f)
			{
				s_DebugSpheres[i].Lifetime -= deltaTime;
				if (s_DebugSpheres[i].Lifetime < 0.0f)
				{
					DeleteSphere(i);
				}
			}
		}
	}

	void DebugDrawing::Render()
	{
		for (size_t i = 0; i < s_LinePointer; ++i)
		{
			RenderLine(s_DebugLines[i]);
		}

		for (size_t i = 0; i < s_SpherePointer; ++i)
		{
			RenderSphere(s_DebugSpheres[i]);
		}
	}

	DebugDrawing::DebugLine DebugDrawing::RayToLine(Ray ray, Color color, float lifeTime)
	{
		DebugLine line;
		line.From = ray.position;
		line.To = Vector3Add(ray.position, ray.direction);
		line.Color = color;
		line.Lifetime = lifeTime;
		return line;
	}

	void DebugDrawing::RenderLine(const DebugLine& line)
	{
		DrawLine3D(line.From, line.To, line.Color);
	}

	void DebugDrawing::RenderSphere(const DebugSphere& sphere)
	{
		if (sphere.IsWireframe)
		{
			DrawSphereWires(sphere.Center, sphere.Radius, 16, 16, sphere.Color);
		}
		else
		{
			DrawSphere(sphere.Center, sphere.Radius, sphere.Color);
		}
	}

	void DebugDrawing::AddLine(DebugLine line)
	{
		if (s_LinePointer < MAX_DEBUG_SHAPES - 1)
		{
			s_DebugLines[s_LinePointer] = line;
			s_LinePointer++;
		}
	}

	void DebugDrawing::AddSphere(DebugSphere sphere)
	{
		if (s_SpherePointer < MAX_DEBUG_SHAPES - 1)
		{
			s_DebugSpheres[s_SpherePointer] = sphere;
			s_SpherePointer++;
		}
	}

	void DebugDrawing::DeleteLine(int index)
	{
		if (index > s_LinePointer)
		{
			return;
		}

		if (index != s_LinePointer)
		{
			std::swap(s_DebugLines[index], s_DebugLines[s_LinePointer]);
		}

		s_LinePointer--;
	}

	void DebugDrawing::DeleteSphere(int index)
	{
		if (index > s_SpherePointer)
		{
			return;
		}

		if (index != s_SpherePointer)
		{
			std::swap(s_DebugSpheres[index], s_DebugSpheres[s_SpherePointer]);
		}

		s_SpherePointer--;
	}

	void DebugDrawing::DrawLightMarker(Color color, Eigen::Vector3f origin, float lifetime)
	{
		using namespace Utils::Math;
		DrawLightMarker(color, ToVector3(origin), lifetime);
	}

	void DebugDrawing::DrawLightMarker(Color color, Vector3 origin, float lifetime)
	{
		if (!s_Flags[DebugFlags::LIGHTS])
		{
			return;
		}
		AddSphere(DebugSphere{ origin, s_MarkerScale, color, lifetime, false });
	}

	void DebugDrawing::DrawDebugLine(Vector3 from, Vector3 to, Color color, float lifetime)
	{
		if (!s_Flags[DebugFlags::CONSTRAINTS])
		{
			return;
		}

		AddLine(DebugLine{ from, to, color, lifetime });
	}

	void DebugDrawing::DrawDebugSphere(Vector3 center, float radius, Color color, float lifetime, bool wireframe)
	{
		if (!s_Flags[DebugFlags::CONSTRAINTS])
		{
			return;
		}

		AddSphere(DebugSphere{ center, radius, color, lifetime, wireframe });
	}

	void DebugDrawing::DrawForceMarker(Color color,
		Eigen::Vector3f origin, Eigen::Quaternionf rotation,
		Eigen::Vector3f forcePosition, Eigen::Vector3f direction,
		bool isLocal, float minLength, float lifetime)
	{
		using namespace Utils::Math;

		if (!s_Flags[DebugFlags::FORCES])
		{
			return;
		}

		if (isLocal)
		{
			forcePosition = origin + rotation.toRotationMatrix() * forcePosition;
			direction = rotation.toRotationMatrix() * direction;
		}

		if (minLength > 0.0f)
		{
			const float minLSq = minLength * minLength;
			if (direction.squaredNorm() < minLSq)
			{
				direction = minLength * direction.normalized();
			}
		}

		if ((forcePosition - origin).squaredNorm() > 0.0f)
		{
			AddLine(DebugLine{ ToVector3(origin), ToVector3(forcePosition), color, lifetime });
		}

		AddLine(RayToLine(Ray{ ToVector3(forcePosition), ToVector3(s_ForceScale * direction) }, color, lifetime));
		AddSphere(DebugSphere{ ToVector3(forcePosition), s_MarkerScale, color, lifetime, false });
	}
}
