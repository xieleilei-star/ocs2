/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <ocs2_core/misc/LoadData.h>

#include "ocs2_legged_robot/common/Types.h"

namespace ocs2 {
namespace legged_robot {

enum ModeNumber {  // {LF, RF, LM, RM, LH, RH}
  FLY = 0,
  RH = 1,
  LH = 2,
  LH_RH = 3,
  RM = 4,
  RM_RH = 5,
  RM_LH = 6,
  RM_LH_RH = 7,
  LM = 8,
  LM_RH = 9,
  LM_LH = 10,
  LM_LH_RH = 11,
  LM_RM = 12,
  LM_RM_RH = 13,
  LM_RM_LH = 14,
  LM_RM_LH_RH = 15,
  RF = 16,
  RF_RH = 17,
  RF_LH = 18,
  RF_LH_RH = 19,
  RF_RM = 20,
  RF_RM_RH = 21,
  RF_RM_LH = 22,
  RF_RM_LH_RH = 23,
  RF_LM = 24,
  RF_LM_RH = 25,
  RF_LM_LH = 26,
  RF_LM_LH_RH = 27,
  RF_LM_RM = 28,
  RF_LM_RM_RH = 29,
  RF_LM_RM_LH = 30,
  RF_LM_RM_LH_RH = 31,
  LF = 32,
  LF_RH = 33,
  LF_LH = 34,
  LF_LH_RH = 35,
  LF_RM = 36,
  LF_RM_RH = 37,
  LF_RM_LH = 38,
  LF_RM_LH_RH = 39,
  LF_LM = 40,
  LF_LM_RH = 41,
  LF_LM_LH = 42,
  LF_LM_LH_RH = 43,
  LF_LM_RM = 44,
  LF_LM_RM_RH = 45,
  LF_LM_RM_LH = 46,
  LF_LM_RM_LH_RH = 47,
  LF_RF = 48,
  LF_RF_RH = 49,
  LF_RF_LH = 50,
  LF_RF_LH_RH = 51,
  LF_RF_RM = 52,
  LF_RF_RM_RH = 53,
  LF_RF_RM_LH = 54,
  LF_RF_RM_LH_RH = 55,
  LF_RF_LM = 56,
  LF_RF_LM_RH = 57,
  LF_RF_LM_LH = 58,
  LF_RF_LM_LH_RH = 59,
  LF_RF_LM_RM = 60,
  LF_RF_LM_RM_RH = 61,
  LF_RF_LM_RM_LH = 62,
  STANCE = 63,
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline contact_flag_t modeNumber2StanceLeg(const size_t& modeNumber) {
  contact_flag_t stanceLegs;  // {LF, RF, LM, RM, LH, RH}
  // stanceLegs.resize(6);

  // Extract bits for each leg: bit0=RH, bit1=LH, bit2=RM, bit3=LM, bit4=RF, bit5=LF
  stanceLegs[0] = (modeNumber & 32) != 0;  // LF
  stanceLegs[1] = (modeNumber & 16) != 0;  // RF
  stanceLegs[2] = (modeNumber & 8) != 0;   // LM
  stanceLegs[3] = (modeNumber & 4) != 0;   // RM
  stanceLegs[4] = (modeNumber & 2) != 0;   // LH
  stanceLegs[5] = (modeNumber & 1) != 0;   // RH

  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t stanceLeg2ModeNumber(const contact_flag_t& stanceLegs) {
  // {LF, RF, LM, RM, LH, RH} -> bit encoding: LF=bit5, RF=bit4, LM=bit3, RM=bit2, LH=bit1, RH=bit0
  return static_cast<size_t>(stanceLegs[5]) +           // RH
         2 * static_cast<size_t>(stanceLegs[4]) +       // LH
         4 * static_cast<size_t>(stanceLegs[3]) +       // RM
         8 * static_cast<size_t>(stanceLegs[2]) +       // LM
         16 * static_cast<size_t>(stanceLegs[1]) +      // RF
         32 * static_cast<size_t>(stanceLegs[0]);       // LF
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::string modeNumber2String(const size_t& modeNumber) {
  // build the map from mode number to name - showing main patterns for 6-legged robot
  std::map<size_t, std::string> modeToName;
  modeToName[FLY] = "FLY";
  modeToName[RH] = "RH";
  modeToName[LH] = "LH";
  modeToName[LH_RH] = "LH_RH";
  modeToName[RM] = "RM";
  modeToName[RM_RH] = "RM_RH";
  modeToName[RM_LH] = "RM_LH";
  modeToName[RM_LH_RH] = "RM_LH_RH";
  modeToName[LM] = "LM";
  modeToName[LM_RH] = "LM_RH";
  modeToName[LM_LH] = "LM_LH";
  modeToName[LM_LH_RH] = "LM_LH_RH";
  modeToName[LM_RM] = "LM_RM";
  modeToName[LM_RM_RH] = "LM_RM_RH";
  modeToName[LM_RM_LH] = "LM_RM_LH";
  modeToName[LM_RM_LH_RH] = "LM_RM_LH_RH";
  modeToName[RF] = "RF";
  modeToName[RF_RH] = "RF_RH";
  modeToName[RF_LH] = "RF_LH";
  modeToName[RF_LH_RH] = "RF_LH_RH";
  modeToName[RF_RM] = "RF_RM";
  modeToName[RF_RM_RH] = "RF_RM_RH";
  modeToName[RF_RM_LH] = "RF_RM_LH";
  modeToName[RF_RM_LH_RH] = "RF_RM_LH_RH";
  modeToName[RF_LM] = "RF_LM";
  modeToName[RF_LM_RH] = "RF_LM_RH";
  modeToName[RF_LM_LH] = "RF_LM_LH";
  modeToName[RF_LM_LH_RH] = "RF_LM_LH_RH";
  modeToName[RF_LM_RM] = "RF_LM_RM";
  modeToName[RF_LM_RM_RH] = "RF_LM_RM_RH";
  modeToName[RF_LM_RM_LH] = "RF_LM_RM_LH";
  modeToName[RF_LM_RM_LH_RH] = "RF_LM_RM_LH_RH";
  modeToName[LF] = "LF";
  modeToName[LF_RH] = "LF_RH";
  modeToName[LF_LH] = "LF_LH";
  modeToName[LF_LH_RH] = "LF_LH_RH";
  modeToName[LF_RM] = "LF_RM";
  modeToName[LF_RM_RH] = "LF_RM_RH";
  modeToName[LF_RM_LH] = "LF_RM_LH";
  modeToName[LF_RM_LH_RH] = "LF_RM_LH_RH";
  modeToName[LF_LM] = "LF_LM";
  modeToName[LF_LM_RH] = "LF_LM_RH";
  modeToName[LF_LM_LH] = "LF_LM_LH";
  modeToName[LF_LM_LH_RH] = "LF_LM_LH_RH";
  modeToName[LF_LM_RM] = "LF_LM_RM";
  modeToName[LF_LM_RM_RH] = "LF_LM_RM_RH";
  modeToName[LF_LM_RM_LH] = "LF_LM_RM_LH";
  modeToName[LF_LM_RM_LH_RH] = "LF_LM_RM_LH_RH";
  modeToName[LF_RF] = "LF_RF";
  modeToName[LF_RF_RH] = "LF_RF_RH";
  modeToName[LF_RF_LH] = "LF_RF_LH";
  modeToName[LF_RF_LH_RH] = "LF_RF_LH_RH";
  modeToName[LF_RF_RM] = "LF_RF_RM";
  modeToName[LF_RF_RM_RH] = "LF_RF_RM_RH";
  modeToName[LF_RF_RM_LH] = "LF_RF_RM_LH";
  modeToName[LF_RF_RM_LH_RH] = "LF_RF_RM_LH_RH";
  modeToName[LF_RF_LM] = "LF_RF_LM";
  modeToName[LF_RF_LM_RH] = "LF_RF_LM_RH";
  modeToName[LF_RF_LM_LH] = "LF_RF_LM_LH";
  modeToName[LF_RF_LM_LH_RH] = "LF_RF_LM_LH_RH";
  modeToName[LF_RF_LM_RM] = "LF_RF_LM_RM";
  modeToName[LF_RF_LM_RM_RH] = "LF_RF_LM_RM_RH";
  modeToName[LF_RF_LM_RM_LH] = "LF_RF_LM_RM_LH";
  modeToName[STANCE] = "STANCE";

  // if mode number is found in map, return its name, otherwise return generic string
  auto it = modeToName.find(modeNumber);
  if (it != modeToName.end()) {
    return it->second;
  } else {
    return "MODE_" + std::to_string(modeNumber);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t string2ModeNumber(const std::string& modeString) {
  // build the map from name to mode number
  std::map<std::string, size_t> nameToMode;
  nameToMode["FLY"] = FLY;
  nameToMode["RH"] = RH;
  nameToMode["LH"] = LH;
  nameToMode["LH_RH"] = LH_RH;
  nameToMode["RM"] = RM;
  nameToMode["RM_RH"] = RM_RH;
  nameToMode["RM_LH"] = RM_LH;
  nameToMode["RM_LH_RH"] = RM_LH_RH;
  nameToMode["LM"] = LM;
  nameToMode["LM_RH"] = LM_RH;
  nameToMode["LM_LH"] = LM_LH;
  nameToMode["LM_LH_RH"] = LM_LH_RH;
  nameToMode["LM_RM"] = LM_RM;
  nameToMode["LM_RM_RH"] = LM_RM_RH;
  nameToMode["LM_RM_LH"] = LM_RM_LH;
  nameToMode["LM_RM_LH_RH"] = LM_RM_LH_RH;
  nameToMode["RF"] = RF;
  nameToMode["RF_RH"] = RF_RH;
  nameToMode["RF_LH"] = RF_LH;
  nameToMode["RF_LH_RH"] = RF_LH_RH;
  nameToMode["RF_RM"] = RF_RM;
  nameToMode["RF_RM_RH"] = RF_RM_RH;
  nameToMode["RF_RM_LH"] = RF_RM_LH;
  nameToMode["RF_RM_LH_RH"] = RF_RM_LH_RH;
  nameToMode["RF_LM"] = RF_LM;
  nameToMode["RF_LM_RH"] = RF_LM_RH;
  nameToMode["RF_LM_LH"] = RF_LM_LH;
  nameToMode["RF_LM_LH_RH"] = RF_LM_LH_RH;
  nameToMode["RF_LM_RM"] = RF_LM_RM;
  nameToMode["RF_LM_RM_RH"] = RF_LM_RM_RH;
  nameToMode["RF_LM_RM_LH"] = RF_LM_RM_LH;
  nameToMode["RF_LM_RM_LH_RH"] = RF_LM_RM_LH_RH;
  nameToMode["LF"] = LF;
  nameToMode["LF_RH"] = LF_RH;
  nameToMode["LF_LH"] = LF_LH;
  nameToMode["LF_LH_RH"] = LF_LH_RH;
  nameToMode["LF_RM"] = LF_RM;
  nameToMode["LF_RM_RH"] = LF_RM_RH;
  nameToMode["LF_RM_LH"] = LF_RM_LH;
  nameToMode["LF_RM_LH_RH"] = LF_RM_LH_RH;
  nameToMode["LF_LM"] = LF_LM;
  nameToMode["LF_LM_RH"] = LF_LM_RH;
  nameToMode["LF_LM_LH"] = LF_LM_LH;
  nameToMode["LF_LM_LH_RH"] = LF_LM_LH_RH;
  nameToMode["LF_LM_RM"] = LF_LM_RM;
  nameToMode["LF_LM_RM_RH"] = LF_LM_RM_RH;
  nameToMode["LF_LM_RM_LH"] = LF_LM_RM_LH;
  nameToMode["LF_LM_RM_LH_RH"] = LF_LM_RM_LH_RH;
  nameToMode["LF_RF"] = LF_RF;
  nameToMode["LF_RF_RH"] = LF_RF_RH;
  nameToMode["LF_RF_LH"] = LF_RF_LH;
  nameToMode["LF_RF_LH_RH"] = LF_RF_LH_RH;
  nameToMode["LF_RF_RM"] = LF_RF_RM;
  nameToMode["LF_RF_RM_RH"] = LF_RF_RM_RH;
  nameToMode["LF_RF_RM_LH"] = LF_RF_RM_LH;
  nameToMode["LF_RF_RM_LH_RH"] = LF_RF_RM_LH_RH;
  nameToMode["LF_RF_LM"] = LF_RF_LM;
  nameToMode["LF_RF_LM_RH"] = LF_RF_LM_RH;
  nameToMode["LF_RF_LM_LH"] = LF_RF_LM_LH;
  nameToMode["LF_RF_LM_LH_RH"] = LF_RF_LM_LH_RH;
  nameToMode["LF_RF_LM_RM"] = LF_RF_LM_RM;
  nameToMode["LF_RF_LM_RM_RH"] = LF_RF_LM_RM_RH;
  nameToMode["LF_RF_LM_RM_LH"] = LF_RF_LM_RM_LH;
  nameToMode["STANCE"] = STANCE;

  auto it = nameToMode.find(modeString);
  if (it != nameToMode.end()) {
    return it->second;
  } else {
    // try to parse generic "MODE_XX" format
    if (modeString.substr(0, 5) == "MODE_") {
      return std::stoul(modeString.substr(5));
    }
    throw std::runtime_error("Unknown mode string: " + modeString);
  }
}

}  // namespace legged_robot
}  // end of namespace ocs2
