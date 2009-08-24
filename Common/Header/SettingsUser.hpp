/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000 - 2009

	M Roberts (original release)
	Robin Birch <robinb@ruffnready.co.uk>
	Samuel Gisiger <samuel.gisiger@triadis.ch>
	Jeff Goodenough <jeff@enborne.f2s.com>
	Alastair Harrison <aharrison@magic.force9.co.uk>
	Scott Penrose <scottp@dd.com.au>
	John Wharington <jwharington@gmail.com>
	Lars H <lars_hn@hotmail.com>
	Rob Dunning <rob@raspberryridgesheepfarm.com>
	Russell King <rmk@arm.linux.org.uk>
	Paolo Ventafridda <coolwind@email.it>
	Tobias Lohner <tobias@lohner-net.de>
	Mirek Jezek <mjezek@ipplc.cz>
	Max Kellermann <max@duempel.org>

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/
#ifndef XCSOAR_SETTINGS_USER_HPP
#define XCSOAR_SETTINGS_USER_HPP

// changed only in config or by user interface /////////////////////////////
// not expected to be used by other threads

// user interface options

// where using these from Calculations or MapWindow thread, should
// protect

// local time adjustment
extern int UTCOffset;

extern bool CircleZoom;
extern bool EnableTopology;
extern bool EnableTerrain;
extern int  FinalGlideTerrain;
extern int  DisplayOrientation;
extern int  DisplayTextType;
extern bool EnableCDICruise;
extern bool EnableCDICircling;
extern bool EnableVarioGauge;
extern int  MenuTimeoutMax;
extern bool EnableAutoBacklight; // VENTA4
extern bool EnableAutoSoundVolume; // VENTA4
extern bool ExtendedVisualGlide;
extern int OnAirSpace; // VENTA3 toggle DrawAirSpace

extern bool VirtualKeys;

// sound stuff not used?
extern bool EnableSoundVario;
extern bool EnableSoundTask;
extern bool EnableSoundModes;
extern int  SoundVolume;
extern int  SoundDeadband;

#endif
