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

#include "InfoBoxes/Content/CombinedTask.hpp"
#include "InfoBoxes/Content/Task.hpp"
#include "InfoBoxes/Panel/Panel.hpp"
#include "InfoBoxes/Data.hpp"
#include "Interface.hpp"
#include "Components.hpp"
#include "Task/ProtectedTaskManager.hpp"
#include "Dialogs/Waypoint/WaypointDialogs.hpp"
#include "Engine/Util/Gradient.hpp"
#include "Engine/Waypoint/Waypoint.hpp"
#include "Units/Units.hpp"
#include "Formatter/TimeFormatter.hpp"
#include "Language/Language.hpp"
#include "Widget/CallbackWidget.hpp"
#include "Util/StringUtil.hpp"
//#include "Util/StaticString.hpp"
#include "UIGlobals.hpp"
#include "Look/Look.hpp"
#include "Formatter/UserUnits.hpp"


#include <tchar.h>
#include <stdio.h>
#include <stdlib.h>



static void
CombinedShowNextWaypointDetails()
{
  if (protected_task_manager == nullptr)
    return;

  const Waypoint *wp = protected_task_manager->GetActiveWaypoint();
  if (wp == nullptr)
    return;

  dlgWaypointDetailsShowModal(*wp);
}

static Widget *
CombinedLoadNextWaypointDetailsPanel(unsigned id)
{
  return new CallbackWidget(CombinedShowNextWaypointDetails);
}

#ifdef __clang__

/* gcc gives "redeclaration differs in 'constexpr'" */

constexpr
#endif
const InfoBoxPanel combined_next_waypoint_infobox_panels[] = {
  { N_("Details"), CombinedLoadNextWaypointDetailsPanel },
  { nullptr, nullptr }
};

void
UpdateInfoBoxCombinedNextaDistance(InfoBoxData &data)
{
  const Waypoint* way_point = protected_task_manager != NULL
    ? protected_task_manager->GetActiveWaypoint()
    : NULL;

  // Set title
  if (!way_point)
    data.SetTitle(_("WP Dist"));
  else
    data.SetTitle(way_point->name.c_str());

  // use proper non-terminal next task stats

  const NMEAInfo &basic = CommonInterface::Basic();
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GeoVector &vector_remaining = task_stats.current_leg.vector_remaining;
  if (!task_stats.task_valid || !vector_remaining.IsValid()) {
    data.SetCommentInvalid();
    return;
  }

  // Set Value
  data.SetCommentFromDistance(vector_remaining.distance);
  data.SetCommentColor(task_stats.inside_oz ? 3 : 0);

  if (basic.track_available) {
    Angle bd = vector_remaining.bearing - basic.track;
    data.SetValueFromBearingDifference(bd);
  } else
    data.SetValueInvalid();
}

static void
SetValueCombinedFromAltDiff(InfoBoxData &data, const TaskStats &task_stats,
                    const GlideResult &solution)
{
  if (!task_stats.task_valid || !solution.IsAchievable()) {
    data.SetValueInvalid();
    return;
  }

  const ComputerSettings &settings = CommonInterface::GetComputerSettings();
  fixed altitude_difference =
    solution.SelectAltitudeDifference(settings.task.glide);
  data.SetValueFromArrival(altitude_difference);
}

void
UpdateInfoBoxCombinedNextAltitudeDiffaGR(InfoBoxData &data)
{
  // pilots want this to be assuming terminal flight to this wp

  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  const GlideResult &next_solution = task_stats.current_leg.solution_remaining;

  SetValueCombinedFromAltDiff(data, task_stats, next_solution);

  // pilots want this to be assuming terminal flight to this wp, and this
  // is what current_leg gradient does.

  if (!CommonInterface::Calculated().task_stats.task_valid) {
    data.SetCommentInvalid();
    return;
  }

  fixed gradient = CommonInterface::Calculated().task_stats.current_leg.gradient;

  if (!positive(gradient)) {
    data.SetComment(_T("+++"));
    return;
  }
  if (::GradientValid(gradient)) {
    data.SetCommentFromGlideRatio(gradient);
  } else {
    data.SetCommentInvalid();
  }
}

void
UpdateInfoBoxCombinedFinalDistanceaETE(InfoBoxData &data)
{
  //FinalDistance
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.task_stats;

  if (!task_stats.task_valid ||
      !task_stats.current_leg.vector_remaining.IsValid() ||
      !task_stats.total.remaining.IsDefined()) {
    data.SetValueInvalid();
    return;
  }

  // Set Value
  data.SetValueFromDistance(task_stats.task_finished
                            ? task_stats.current_leg.vector_remaining.distance
                            : task_stats.total.remaining.GetDistance());
  //ETE
 if (!task_stats.task_valid || !task_stats.total.IsAchievable()) {
   data.SetCommentInvalid();
   return;
 }

   assert(!negative(task_stats.total.time_remaining_now));

   TCHAR comment[32];
   TCHAR commentb[32];
   const int dd = abs((int)task_stats.total.time_remaining_now);
   FormatTimeTwoLines( comment, commentb, dd);
   data.SetComment(comment);

}

void
UpdateInfoBoxCombinedFinalDistanceaETA(InfoBoxData &data)
{
  //FinalDistance
  const auto &calculated = CommonInterface::Calculated();
  const TaskStats &task_stats = calculated.task_stats;

  if (!task_stats.task_valid ||
      !task_stats.current_leg.vector_remaining.IsValid() ||
      !task_stats.total.remaining.IsDefined()) {
    data.SetValueInvalid();
    return;
  }

  // Set Value
  data.SetValueFromDistance(task_stats.task_finished
                            ? task_stats.current_leg.vector_remaining.distance
                            : task_stats.total.remaining.GetDistance());

  //ETA
  const BrokenTime &now_local = CommonInterface::Calculated().date_time_local;

  if (!task_stats.task_valid || !task_stats.total.IsAchievable() ||
      !now_local.IsPlausible()) {
    data.SetCommentInvalid();
    return;
  }

  const BrokenTime t = now_local +
  unsigned(task_stats.total.solution_remaining.time_elapsed);

   // Set Comment
   data.UnsafeFormatComment(_T("%02u:%02u:%02u"), t.hour, t.minute, t.second);

}

void
UpdateInfoBoxCombinedTaskSpeedaPercentClimb(InfoBoxData &data)
{
  // TaskSpeed / Value
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid || !task_stats.total.travelled.IsDefined()) {
    data.SetValueInvalid();
    return;
  }

  data.SetValue(_T("%2.0f"),
                    Units::ToUserTaskSpeed(task_stats.total.travelled.GetSpeed()));

  // Set Unit
  data.SetValueUnit(Units::current.task_speed_unit);

  //PercentClimb / Comment
  if (negative(CommonInterface::Calculated().circling_percentage))
    data.SetCommentInvalid();
  else
    data.UnsafeFormatComment(_T("%2.0f%%"),
                  CommonInterface::Calculated().circling_percentage);
}

void
UpdateInfoBoxCombinedAvgGRaFinalGR(InfoBoxData &data)
{
  //GR Average
  const fixed average_gr = CommonInterface::Calculated().average_gr;

    if (average_gr == fixed(0)) {
      data.SetValueInvalid();
      return;
    }

    // Set Value
    if (average_gr < fixed(0))
      data.SetValue(_T("^^^"));
    else if (!::GradientValid(average_gr))
      data.SetValue(_T("+++"));
    else
      data.SetValueFromGlideRatio(average_gr);

  //GR Final
  const TaskStats &task_stats = CommonInterface::Calculated().task_stats;
  if (!task_stats.task_valid) {
    data.SetCommentInvalid();
    return;
  }

  fixed gradient = task_stats.total.gradient;

  if (!positive(gradient)) {
    data.SetComment(_T("+++"));
    return;
  }
  if (::GradientValid(gradient))
    data.SetCommentFromGlideRatio(gradient);
  else
    data.SetCommentInvalid();
}
