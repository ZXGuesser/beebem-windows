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

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <assert.h>

#include "JoystickController.h"
#include "DebugTrace.h"

// #define DEBUG_JOYSTICK

/****************************************************************************/

#ifdef DEBUG_JOYSTICK

static const char* DInputErrorStr(HRESULT hResult)
{
	switch (hResult)
	{
		case DIERR_DEVICENOTREG:
			return "DIERR_DEVICENOTREG";

		case DIERR_INVALIDPARAM:
			return "DIERR_INVALIDPARAM";

		case DIERR_NOINTERFACE:
			return "DIERR_NOINTERFACE";

		case DIERR_NOTINITIALIZED:
			return "DIERR_NOTINITIALIZED";

		case DIERR_OUTOFMEMORY:
			return "DIERR_OUTOFMEMORY";

		case DIERR_INPUTLOST:
			return "DIERR_INPUTLOST";

		case DIERR_NOTACQUIRED:
			return "DIERR_NOTACQUIRED";

		case E_PENDING:
			return "E_PENDING";

		default:
			return "?";
	}
}

#endif

/****************************************************************************/

JoystickDeviceInfo::JoystickDeviceInfo() :
	Type(JoystickDeviceType::XInput),
	XInputIndex(0)
{
	ZeroMemory(&DirectInputGuid, sizeof(DirectInputGuid));
}

/****************************************************************************/

JoystickController::JoystickController() :
	m_hInstance(nullptr),
	m_hWnd(nullptr),
	m_hXInputModule(nullptr),
	m_XInputGetState(nullptr),
	m_XInputSetState(nullptr),
	m_pDirectInput(nullptr),
	m_pDirectInputDevice(nullptr),
	m_ActiveDevice(0),
	m_MinX(0),
	m_MaxX(65535),
	m_MinY(0),
	m_MaxY(65535)
{
}

/****************************************************************************/

JoystickController::~JoystickController()
{
	if (m_hXInputModule != nullptr)
	{
		FreeLibrary(m_hXInputModule);
		m_hXInputModule = nullptr;
	}

	if (m_pDirectInputDevice != nullptr)
	{
		m_pDirectInputDevice->Release();
		m_pDirectInputDevice = nullptr;
	}

	if (m_pDirectInput != nullptr)
	{
		m_pDirectInput->Release();
		m_pDirectInput = nullptr;
	}
}

/****************************************************************************/

bool JoystickController::Init(HINSTANCE hInstance, HWND hWnd)
{
	assert(m_hInstance == nullptr);
	assert(m_hWnd == nullptr);

	m_hInstance = hInstance;
	m_hWnd = hWnd;

	static const char* XInputLibraries[] =
	{
		"xinput1_4.dll",
		"xinput1_3.dll",
		"xinput9_1_0.dll"
	};

	for (size_t i = 0; i < _countof(XInputLibraries); i++)
	{
		m_hXInputModule = LoadLibrary(XInputLibraries[i]);

		if (m_hXInputModule != nullptr)
		{
			m_XInputGetState = reinterpret_cast<XINPUT_GET_STATE>(
				GetProcAddress(m_hXInputModule, "XInputGetState")
			);

			m_XInputSetState = reinterpret_cast<XINPUT_SET_STATE>(
				GetProcAddress(m_hXInputModule, "XInputSetState")
			);
			break;
		}
	}

	DirectInput8Create(m_hInstance,
	                   DIRECTINPUT_VERSION,
	                   IID_IDirectInput8,
	                   reinterpret_cast<void**>(&m_pDirectInput),
	                   nullptr);

	return m_hXInputModule != nullptr || m_pDirectInput != nullptr;
}

/****************************************************************************/

void JoystickController::EnumerateDevices()
{
	m_Devices.clear();

	EnumerateXInputDevices();
	EnumerateDirectInputDevices();
}

/****************************************************************************/

void JoystickController::EnumerateXInputDevices()
{
	if (m_hXInputModule == nullptr)
	{
		return;
	}

	for (DWORD i = 0; i < XUSER_MAX_COUNT; ++i)
	{
		XINPUT_STATE State;

		if (m_XInputGetState(i, &State) == ERROR_SUCCESS)
		{
			JoystickDeviceInfo Info;

			Info.Type = JoystickDeviceType::XInput;
			Info.Name = "XBox Controller - Slot " + std::to_string(i) + " [XInput]";
			Info.XInputIndex = i;

			m_Devices.emplace_back(Info);
		}
	}
}

/****************************************************************************/

void JoystickController::EnumerateDirectInputDevices()
{
	if (m_pDirectInput == nullptr)
	{
		return;
	}

	m_pDirectInput->EnumDevices(DI8DEVCLASS_GAMECTRL,
	                            EnumDInputCallback,
	                            this,
	                            DIEDFL_ATTACHEDONLY);
}

/****************************************************************************/

BOOL CALLBACK JoystickController::EnumDInputCallback(const DIDEVICEINSTANCE* pDeviceInstance,
                                                     void* pContext)
{
	JoystickController* pController = reinterpret_cast<JoystickController*>(pContext);

	return pController->EnumDInputCallback(pDeviceInstance);
}

/****************************************************************************/

BOOL JoystickController::EnumDInputCallback(const DIDEVICEINSTANCE* pDeviceInstance)
{
	JoystickDeviceInfo Info;

	Info.Type = JoystickDeviceType::DirectInput;
	Info.Name = std::string(pDeviceInstance->tszProductName) + " [DirectInput]";
	Info.DirectInputGuid = pDeviceInstance->guidInstance;

	m_Devices.emplace_back(Info);

	return DIENUM_CONTINUE;
}

/****************************************************************************/

size_t JoystickController::GetDeviceCount() const
{
	return m_Devices.size();
}

/****************************************************************************/

const JoystickDeviceInfo& JoystickController::GetDeviceInfo(size_t Index) const
{
	return m_Devices[Index];
}

/****************************************************************************/

size_t JoystickController::GetActiveDevice() const
{
	return m_ActiveDevice;
}

/****************************************************************************/

void JoystickController::SetActiveDevice(size_t Index)
{
	ReleaseActiveDevice();

	if (Index < m_Devices.size())
	{
		m_ActiveDevice = Index;

		AcquireActiveDevice();
	}
	else
	{
		m_ActiveDevice = 0;
	}
}

/****************************************************************************/

void JoystickController::ReleaseActiveDevice()
{
	if (m_pDirectInputDevice != nullptr)
	{
		m_pDirectInputDevice->Release();
		m_pDirectInputDevice = nullptr;
	}
}

/****************************************************************************/

void JoystickController::AcquireActiveDevice()
{
	#ifdef DEBUG_JOYSTICK
	DebugTrace("JoystickController::AcquireActiveDevice()\n");
	#endif

	const JoystickDeviceInfo& DeviceInfo = m_Devices[m_ActiveDevice];

	if (DeviceInfo.Type == JoystickDeviceType::DirectInput)
	{
		HRESULT hResult = m_pDirectInput->CreateDevice(DeviceInfo.DirectInputGuid,
		                                               &m_pDirectInputDevice,
		                                               nullptr);

		if (FAILED(hResult))
		{
			#ifdef DEBUG_JOYSTICK
			DebugTrace("m_pDirectInput->CreateDevice() returned %s\n", DInputErrorStr(hResult));
			#endif

			return;
		}

        hResult = m_pDirectInputDevice->SetDataFormat(&c_dfDIJoystick);

		if (FAILED(hResult))
		{
			#ifdef DEBUG_JOYSTICK
			DebugTrace("m_pDirectInputDevice->SetDataFormat() returned %s\n", DInputErrorStr(hResult));
			#endif

			m_pDirectInputDevice->Release();
			m_pDirectInputDevice = nullptr;
			return;
		}

		hResult = m_pDirectInputDevice->SetCooperativeLevel(m_hWnd,
		                                                    DISCL_FOREGROUND | DISCL_NONEXCLUSIVE);

		if (FAILED(hResult))
		{
			#ifdef DEBUG_JOYSTICK
			DebugTrace("m_pDirectInputDevice->SetCooperativeLevel() returned %s\n", DInputErrorStr(hResult));
			#endif

			m_pDirectInputDevice->Release();
			m_pDirectInputDevice = nullptr;
			return;
		}

		DIPROPRANGE Range;
		ZeroMemory(&Range, sizeof(Range));
		Range.diph.dwSize = sizeof(DIPROPRANGE);
		Range.diph.dwHeaderSize = sizeof(DIPROPHEADER);
		Range.diph.dwHow = DIPH_BYOFFSET;
		Range.diph.dwObj = DIJOFS_X;

		hResult = m_pDirectInputDevice->GetProperty(DIPROP_RANGE, &Range.diph);

		if (SUCCEEDED(hResult))
		{
			m_MinX = Range.lMin;
			m_MaxX = Range.lMax;
		}

		ZeroMemory(&Range, sizeof(Range));
		Range.diph.dwSize = sizeof(DIPROPRANGE);
		Range.diph.dwHeaderSize = sizeof(DIPROPHEADER);
		Range.diph.dwHow = DIPH_BYOFFSET;
		Range.diph.dwObj = DIJOFS_Y;

		hResult = m_pDirectInputDevice->GetProperty(DIPROP_RANGE, &Range.diph);

		if (SUCCEEDED(hResult))
		{
			m_MinY = Range.lMin;
			m_MaxY = Range.lMax;
		}

		hResult = m_pDirectInputDevice->Acquire();

		if (FAILED(hResult))
		{
			#ifdef DEBUG_JOYSTICK
			DebugTrace("m_pDirectInputDevice->Acquire() returned %s\n", DInputErrorStr(hResult));
			#endif
		}
	}
}

/****************************************************************************/

static int NormalizeRange(int Value, int Min, int Max)
{
    return ((Value - Min) * 65535LL) / (Max - Min);
}

/****************************************************************************/

bool JoystickController::GetState(JoystickState* pState) const
{
	const JoystickDeviceInfo& DeviceInfo = m_Devices[m_ActiveDevice];

	if (DeviceInfo.Type == JoystickDeviceType::XInput)
	{
		XINPUT_STATE State;

		DWORD Result = m_XInputGetState(DeviceInfo.XInputIndex, &State);

		if (Result != ERROR_SUCCESS)
		{
			#ifdef DEBUG_JOYSTICK
			DebugTrace("JoystickController::GetState()\n");
			DebugTrace("XInputGetState returned %u\n", Result);
			#endif

			return false;
		}

		pState->X = 32767 - (int)State.Gamepad.sThumbLX;
		pState->Y = (int)State.Gamepad.sThumbLY + 32768;
		pState->Buttons = State.Gamepad.wButtons;
	}
	else // DirectInput
	{
		if (m_pDirectInputDevice == nullptr)
		{
			#ifdef DEBUG_JOYSTICK
			DebugTrace("JoystickController::GetState()\n");
			DebugTrace("m_pDirectInputDevice is NULL\n");
			#endif

			return false;
		}

		HRESULT hResult = m_pDirectInputDevice->Poll();

		if (FAILED(hResult))
		{
			#ifdef DEBUG_JOYSTICK
			DebugTrace("JoystickController::GetState()\n");
			DebugTrace("m_pDirectInputDevice->Poll() returned %s\n", DInputErrorStr(hResult));
			#endif

			return hResult == DIERR_NOTACQUIRED;
		}

		DIJOYSTATE State;

		hResult = m_pDirectInputDevice->GetDeviceState(sizeof(State), &State);

		if (FAILED(hResult))
		{
			#ifdef DEBUG_JOYSTICK
			DebugTrace("JoystickController::GetState()\n");
			DebugTrace("m_pDirectInputDevice->GetDeviceState() returned %s\n", DInputErrorStr(hResult));
			#endif

			return hResult == DIERR_NOTACQUIRED;
		}

		pState->X = 65535 - NormalizeRange(State.lX, m_MinX, m_MaxX);
		pState->Y = 65535 - NormalizeRange(State.lY, m_MinY, m_MaxY);
		pState->Buttons = 0;

		for (int i = 0; i < 4; i++)
		{
			if (State.rgbButtons[i] & 0x80)
			{
				pState->Buttons |= JOYSTICK_BUTTON_A << i;
			}
		}

		switch (State.rgdwPOV[0])
		{
			case 0:
				pState->Buttons |= JOYSTICK_BUTTON_UP;
				break;

			case 4500:
				pState->Buttons |= JOYSTICK_BUTTON_RIGHT | JOYSTICK_BUTTON_UP;
				break;

			case 9000:
				pState->Buttons |= JOYSTICK_BUTTON_RIGHT;
				break;

			case 13500:
				pState->Buttons |= JOYSTICK_BUTTON_RIGHT | JOYSTICK_BUTTON_DOWN;
				break;

			case 18000:
				pState->Buttons |= JOYSTICK_BUTTON_DOWN;
				break;

			case 22500:
				pState->Buttons |= JOYSTICK_BUTTON_LEFT | JOYSTICK_BUTTON_DOWN;
				break;

			case 27000:
				pState->Buttons |= JOYSTICK_BUTTON_LEFT;
				break;

			case 31500:
				pState->Buttons |= JOYSTICK_BUTTON_LEFT | JOYSTICK_BUTTON_UP;
				break;
		}
	}

	return true;
}

/****************************************************************************/

void JoystickController::Acquire(bool Acquire)
{
	#ifdef DEBUG_JOYSTICK
	DebugTrace("JoystickController::Acquire(%d)\n", (int)Acquire);
	#endif

	if (m_pDirectInputDevice != nullptr)
	{
		if (Acquire)
		{
			HRESULT hResult = m_pDirectInputDevice->Acquire();

			if (FAILED(hResult))
			{
				#ifdef DEBUG_JOYSTICK
				DebugTrace("m_pDirectInputDevice->Acquire() returned %s\n", DInputErrorStr(hResult));
				#endif
			}
		}
		else
		{
			HRESULT hResult = m_pDirectInputDevice->Unacquire();

			if (FAILED(hResult))
			{
				#ifdef DEBUG_JOYSTICK
				DebugTrace("m_pDirectInputDevice->Unacquire() returned %s\n", DInputErrorStr(hResult));
				#endif
			}
		}
	}
}

/****************************************************************************/
