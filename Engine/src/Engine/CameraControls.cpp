#include "CameraControls.h"
#include "raylib.h"
#include "rcamera.h"
#include "raymath.h"

namespace Utils::Camera
{
	const float CAMERA_MOVE_SPEED = 0.09f;
	const float CAMERA_ROTATION_SPEED = 0.03f;
	const float CAMERA_MOUSE_MOVE_SENSITIVITY = 0.003f; // TODO: it should be independant of framerate
	const float CAMERA_MOUSE_SCROLL_SENSITIVITY = 1.5f;


	bool UpdateCamera(Camera3D* camera, bool rightClickToEdit, bool zoomWithoutControl)
	{
		bool canEdit = true;
		bool cameraMoved = false;

		Vector2 mousePositionDelta = GetMouseDelta();
		const float wheelMovement = -GetMouseWheelMove();

		if (rightClickToEdit)
		{
			if (IsMouseButtonPressed(MOUSE_BUTTON_RIGHT))
			{
				DisableCursor();
			}
			else if (IsMouseButtonReleased(MOUSE_BUTTON_RIGHT))
			{
				EnableCursor();
			}

			canEdit = IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
		}

		if (canEdit)
		{
			bool moveInWorldPlane = false;
			bool rotateAroundTarget = false;
			bool lockView = true;
			bool rotateUp = false;

			// Camera rotation
			if (IsKeyDown(KEY_DOWN))
			{
				CameraPitch(camera, -CAMERA_ROTATION_SPEED, lockView, rotateAroundTarget, rotateUp);
				cameraMoved |= true;
			}

			if (IsKeyDown(KEY_UP))
			{
				CameraPitch(camera, CAMERA_ROTATION_SPEED, lockView, rotateAroundTarget, rotateUp);
				cameraMoved |= true;
			}

			if (IsKeyDown(KEY_RIGHT))
			{
				CameraYaw(camera, -CAMERA_ROTATION_SPEED, rotateAroundTarget);
				cameraMoved |= true;
			}

			if (IsKeyDown(KEY_LEFT))
			{
				CameraYaw(camera, CAMERA_ROTATION_SPEED, rotateAroundTarget);
				cameraMoved |= true;
			}

			if (IsKeyDown(KEY_Q))
			{
				CameraMoveUp(camera, -CAMERA_MOVE_SPEED);
				cameraMoved |= true;
			}

			if (IsKeyDown(KEY_E))
			{
				CameraMoveUp(camera, CAMERA_MOVE_SPEED);
				cameraMoved |= true;
			}

			CameraYaw(camera, -mousePositionDelta.x * CAMERA_MOUSE_MOVE_SENSITIVITY, rotateAroundTarget);
			CameraPitch(camera, -mousePositionDelta.y * CAMERA_MOUSE_MOVE_SENSITIVITY, lockView, rotateAroundTarget, rotateUp);

			// Camera movement
			if (IsKeyDown(KEY_W))
			{
				CameraMoveForward(camera, CAMERA_MOVE_SPEED, moveInWorldPlane);
				cameraMoved |= true;
			}
			if (IsKeyDown(KEY_A))
			{
				CameraMoveRight(camera, -CAMERA_MOVE_SPEED, moveInWorldPlane);
				cameraMoved |= true;
			}
			if (IsKeyDown(KEY_S))
			{
				CameraMoveForward(camera, -CAMERA_MOVE_SPEED, moveInWorldPlane);
				cameraMoved |= true;
			}
			if (IsKeyDown(KEY_D))
			{
				CameraMoveRight(camera, CAMERA_MOVE_SPEED, moveInWorldPlane);
				cameraMoved |= true;
			}
		}

		if (zoomWithoutControl || canEdit)
		{
			// Zoom target distance	
			CameraMoveToTarget(camera, wheelMovement);
			if (wheelMovement != 0)
			{
				cameraMoved |= true;
			}
		}

		cameraMoved |= canEdit && ((mousePositionDelta.x != 0)|| (mousePositionDelta.y != 0) || (wheelMovement != 0));
		return cameraMoved;
	}
}