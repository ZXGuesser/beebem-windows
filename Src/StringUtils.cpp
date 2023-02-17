/****************************************************************
BeebEm - BBC Micro and Master 128 Emulator
Copyright (C) 2020  Chris Needham

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

#include "StringUtils.h"

#include <algorithm>
#include <cctype>

static void trimLeft(std::string& str)
{
	auto pos = std::find_if(str.begin(), str.end(), [](int ch) {
		return !std::isspace(ch);
	});

	str.erase(str.begin(), pos);
}

static void trimRight(std::string& str)
{
	auto pos = std::find_if(str.rbegin(), str.rend(), [](int ch) {
		return !std::isspace(ch);
	});

	str.erase(pos.base(), str.end());
}

void trim(std::string& str)
{
	trimLeft(str);
	trimRight(str);
}

bool ParseNumber(const std::string& str, int* pValue)
{
	if (str.empty())
	{
		return false;
	}

	int Value = 0;

	for (char c : str)
	{
		if (isdigit(c))
		{
			if (Value >= INT_MAX / 10)
			{
				return false;
			}

			Value *= 10;
			Value += c - '0';
		}
		else
		{
			return false;
		}
	}

	*pValue = Value;

	return true;
}
