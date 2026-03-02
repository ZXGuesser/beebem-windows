/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2026  Chris Needham

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public
License along with this program; if not, write to the Free
Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
Boston, MA  02110-1301, USA.
****************************************************************/

#ifndef JOYSTICK_CONTROLLER_HEADER
#define JOYSTICK_CONTROLLER_HEADER

#include <xinput.h>

#define DIRECTINPUT_VERSION 0x0800
#include <dinput.h>

#include <string>
#include <vector>

enum class JoystickDeviceType
{
    XInput,
    DirectInput
};

class JoystickDeviceInfo
{
	public:
		JoystickDeviceType Type;
		std::string Name;
		DWORD XInputIndex;
		GUID DirectInputGuid;

	public:
		JoystickDeviceInfo();
};

// Same as XINPUT_STATE values

constexpr int JOYSTICK_BUTTON_UP    = 0x0001;
constexpr int JOYSTICK_BUTTON_DOWN  = 0x0002;
constexpr int JOYSTICK_BUTTON_LEFT  = 0x0004;
constexpr int JOYSTICK_BUTTON_RIGHT = 0x0008;
constexpr int JOYSTICK_BUTTON_DPAD  = 0x000F;

constexpr int JOYSTICK_BUTTON_A       = 0x1000;
constexpr int JOYSTICK_BUTTON_B       = 0x2000;
constexpr int JOYSTICK_BUTTON_X       = 0x4000;
constexpr int JOYSTICK_BUTTON_Y       = 0x8000;
constexpr int JOYSTICK_BUTTON_BUTTONS = 0xF000;

class JoystickState
{
	public:
		int X; // 0 (right) to 65535 (left)
		int Y; // 0 (down) to 65535 (up)
		int Buttons;
};

class JoystickController
{
	public:
		JoystickController();
		JoystickController(const JoystickController&) = delete;
		JoystickController& operator=(const JoystickController&) = delete;
		~JoystickController();

	public:
		bool Init(HINSTANCE hInstance, HWND hWnd);

		void EnumerateDevices();

		size_t GetDeviceCount() const;
		const JoystickDeviceInfo& GetDeviceInfo(size_t Index) const;

		size_t GetActiveDevice() const;
		void SetActiveDevice(size_t Index);

		bool GetState(JoystickState* pState) const;

		void Acquire(bool Acquire);

	private:
		void EnumerateXInputDevices();
		void EnumerateDirectInputDevices();
		static BOOL CALLBACK EnumDInputCallback(const DIDEVICEINSTANCE* pDeviceInstance,
		                                        void* pContext);
		BOOL EnumDInputCallback(const DIDEVICEINSTANCE* pDeviceInstance);

		void AcquireActiveDevice();
		void ReleaseActiveDevice();

	private:
		HINSTANCE m_hInstance;
		HWND m_hWnd;
		HMODULE m_hXInputModule;

		typedef DWORD (WINAPI *XINPUT_GET_STATE)(DWORD, XINPUT_STATE*);
		typedef DWORD (WINAPI *XINPUT_SET_STATE)(DWORD, XINPUT_VIBRATION*);

		XINPUT_GET_STATE m_XInputGetState;
		XINPUT_SET_STATE m_XInputSetState;

		IDirectInput8* m_pDirectInput;
		IDirectInputDevice8* m_pDirectInputDevice;
		int m_MinX;
		int m_MaxX;
		int m_MinY;
		int m_MaxY;

		std::vector<JoystickDeviceInfo> m_Devices;
		size_t m_ActiveDevice;
};

#endif
