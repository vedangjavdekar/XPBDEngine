#pragma once

struct Camera3D;

namespace Utils::Camera
{
	bool UpdateCamera(Camera3D* camera, bool rightClickToEdit = true, bool zoomWithoutControl = true);		// Returns true if there was camera motion
}