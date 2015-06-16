/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2015 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

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

#include "InfoBoxes/Content/CombinedAltitude.hpp"

#include "Factory.hpp"
#include "InfoBoxes/Data.hpp"
#include "InfoBoxes/Panel/Panel.hpp"
#include "InfoBoxes/Panel/AltitudeInfo.hpp"
#include "InfoBoxes/Panel/AltitudeSimulator.hpp"
#include "InfoBoxes/Panel/AltitudeSetup.hpp"
#include "Units/Units.hpp"
#include "Interface.hpp"
#include "Components.hpp"
#include "Engine/Waypoint/Waypoint.hpp"
#include "Engine/Waypoint/Waypoints.hpp"
#include "Language/Language.hpp"
#include "Blackboard/DeviceBlackboard.hpp"
#include "Components.hpp"
#include "Simulator.hpp"

#include <tchar.h>
#include <stdio.h>

#ifdef __clang__

/* gcc gives "redeclaration differs in 'constexpr'" */

constexpr
#endif

const InfoBoxPanel combinedaltitude_infobox_panels[] = {
  { N_("Simulator"), LoadAltitudeSimulatorPanel },
  { N_("Info"), LoadAltitudeInfoPanel },
  { N_("Setup"), LoadAltitudeSetupPanel },
  { nullptr, nullptr }
};


const InfoBoxPanel *
InfoBoxContentCombinedAltitude::GetDialogContent() {
  return combinedaltitude_infobox_panels;
}

void
UpdateInfoBoxCombinedAltitudeAGLaFL(InfoBoxData &data)
{
  // Altitude AGL / Value
  const DerivedInfo &calculated = CommonInterface::Calculated();
  const NMEAInfo &basic = CommonInterface::Basic();
  const ComputerSettings &settings_computer =
    CommonInterface::GetComputerSettings();

  if (!calculated.altitude_agl_valid) {
    data.SetValueInvalid();
    return;
  }

  data.SetValueFromAltitude(calculated.altitude_agl);
  data.SetValueColor(calculated.altitude_agl <
      CommonInterface::GetComputerSettings().task.route_planner.safety_height_terrain ? 1 : 0);

 // FlightLevel / Comment
 if (basic.pressure_altitude_available) {
    fixed Altitude = Units::ToUserUnit(basic.pressure_altitude, Unit::FEET);

    data.UnsafeFormatComment(_T("%03dFL"), iround(Altitude / 100));

  } else if (basic.gps_altitude_available &&
             settings_computer.pressure_available) {
    // Take gps altitude as baro altitude. This is inaccurate but still fits our needs.
    const AtmosphericPressure &qnh = settings_computer.pressure;
    fixed Altitude = Units::ToUserUnit(qnh.QNHAltitudeToPressureAltitude(basic.gps_altitude), Unit::FEET);

    data.UnsafeFormatComment(_T("%03dFL"), iround(Altitude / 100));

  } else if ((basic.baro_altitude_available || basic.gps_altitude_available) &&
             !settings_computer.pressure_available) {
    data.SetComment(_("no QNH"));
  } else {
    data.SetCommentInvalid();
  }
}

