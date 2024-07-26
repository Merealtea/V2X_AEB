/**
 * @file track_smoother.h
 * @author Hongxin Chen (angleochen@sjtu.edu.cn)
 * @brief
 * @version 0.1
 * @date 2022-08-01
 *
 * @copyright Copyright (c) 2020 CyberC3 Authors. All Rights Reserved.
 *
 */

#pragma once

#include <list>
#include "common.h"
#include "discretized_path.h"

namespace cyberc3 {
namespace planning {
bool SmoothPlatoonTrack(const std::list<TrackPoint> &track_in,
                        DiscretizedPath &track_out);
}
} // namespace cyberc3
