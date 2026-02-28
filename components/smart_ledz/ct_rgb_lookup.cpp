#include "ct_rgb_lookup.h"

#ifdef USE_ESP32

#include <algorithm>
#include <cmath>
#include <limits>

namespace esphome {
namespace smart_ledz {

namespace {

uint8_t clip_and_round_channel_(double value) {
  const auto rounded = static_cast<int>(std::round(value));
  if (rounded < 0) {
    return 0;
  }
  if (rounded > 255) {
    return 255;
  }
  return static_cast<uint8_t>(rounded);
}

double clamp_cct_(double cct) {
  return std::max(1800.0, std::min(12000.0, cct));
}

double clamp_duv_(double duv) {
  return std::max(-6.0, std::min(6.0, duv));
}

}  // namespace

std::array<uint8_t, 3> lookup_ct_rgb(uint16_t kelvin, int8_t duv) {
  const double cct = clamp_cct_(static_cast<double>(kelvin));
  const double y = clamp_duv_(static_cast<double>(duv));
  const double x = std::log(cct);

  const double x2 = x * x;
  const double x3 = x2 * x;
  const double x4 = x3 * x;
  const double y2 = y * y;
  const double y3 = y2 * y;
  const double y4 = y3 * y;
  const double xy = x * y;
  const double x2y = x2 * y;
  const double xy2 = x * y2;
  const double x3y = x3 * y;
  const double x2y2 = x2 * y2;
  const double xy3 = x * y3;

  double red = 0.0;
  double green = 0.0;
  double blue = 0.0;

  if (cct <= 3000.0) {
    red = -1460060.1241 + (748895.5863 * x) + (-7039.7939 * y) + (-143815.6025 * x2) + (2786.0392 * xy) +
          (-10.4107 * y2) + (12259.9927 * x3) + (-366.6168 * x2y) + (3.0577 * xy2) + (-0.0147 * y3) +
          (-391.5756 * x4) + (16.0402 * x3y) + (-0.2199 * x2y2) + (0.0021 * xy3) + (-0.0003 * y4);

    green = 2464661.6467 + (-1263780.3346 * x) + (21354.2706 * y) + (242677.8391 * x2) + (-8386.3163 * xy) +
            (3.1897 * y2) + (-20684.4749 * x3) + (1096.2330 * x2y) + (-1.5240 * xy2) + (0.1269 * y3) +
            (660.3510 * x4) + (-47.6913 * x3y) + (0.1406 * x2y2) + (-0.0168 * xy3) + (0.0013 * y4);

    blue = -64200.4385 + (32989.1626 * x) + (-1403.7206 * y) + (-6279.6030 * x2) + (547.3064 * xy) +
           (-13.3301 * y2) + (523.3536 * x3) + (-71.1217 * x2y) + (3.3815 * xy2) + (-0.0696 * y3) +
           (-16.0503 * x4) + (3.0757 * x3y) + (-0.2148 * x2y2) + (0.0090 * xy3) + (0.0004 * y4);
  } else {
    red = -21816.8020 + (14200.9187 * x) + (-178.3631 * y) + (-3070.5719 * x2) + (72.2238 * xy) +
          (-3.7619 * y2) + (276.4690 * x3) + (-9.5512 * x2y) + (0.8981 * xy2) + (-0.0234 * y3) +
          (-8.9594 * x4) + (0.4091 * x3y) + (-0.0533 * x2y2) + (0.0028 * xy3) + (-0.0006 * y4);

    green = -107374.1087 + (42018.5859 * x) + (-901.8033 * y) + (-5975.8982 * x2) + (299.6568 * xy) +
            (2.0357 * y2) + (363.0634 * x3) + (-32.7887 * x2y) + (-0.4927 * xy2) + (0.0139 * y3) +
            (-7.8267 * x4) + (1.1867 * x3y) + (0.0299 * x2y2) + (-0.0017 * xy3) + (0.0001 * y4);

    blue = 128220.0114 + (-55593.6193 * x) + (861.7219 * y) + (8931.7263 * x2) + (-295.8123 * xy) +
           (2.1700 * y2) + (-630.1880 * x3) + (33.5286 * x2y) + (-0.4935 * xy2) + (-0.0114 * y3) +
           (16.5011 * x4) + (-1.2563 * x3y) + (0.0279 * x2y2) + (0.0013 * xy3);
  }

  return {clip_and_round_channel_(red), clip_and_round_channel_(green), clip_and_round_channel_(blue)};
}

uint16_t estimate_cct_from_rgb(uint8_t target_red, uint8_t target_green, uint8_t target_blue, int8_t duv) {
  int best_error = std::numeric_limits<int>::max();
  uint16_t best_cct = 1800;

  for (uint16_t cct = 1800; cct <= 12000; cct += 100) {
    const auto rgb = lookup_ct_rgb(cct, duv);
    const int dr = static_cast<int>(rgb[0]) - static_cast<int>(target_red);
    const int dg = static_cast<int>(rgb[1]) - static_cast<int>(target_green);
    const int db = static_cast<int>(rgb[2]) - static_cast<int>(target_blue);
    const int error = dr * dr + dg * dg + db * db;
    if (error < best_error) {
      best_error = error;
      best_cct = cct;
    }
  }

  const uint16_t search_start = best_cct < 1900 ? 1800 : static_cast<uint16_t>(best_cct - 100);
  const uint16_t search_end = best_cct > 11900 ? 12000 : static_cast<uint16_t>(best_cct + 100);

  for (uint16_t cct = search_start; cct <= search_end; cct++) {
    const auto rgb = lookup_ct_rgb(cct, duv);
    const int dr = static_cast<int>(rgb[0]) - static_cast<int>(target_red);
    const int dg = static_cast<int>(rgb[1]) - static_cast<int>(target_green);
    const int db = static_cast<int>(rgb[2]) - static_cast<int>(target_blue);
    const int error = dr * dr + dg * dg + db * db;
    if (error < best_error) {
      best_error = error;
      best_cct = cct;
      if (error == 0) {
        break;
      }
    }
  }

  return best_cct;
}

}  // namespace smart_ledz
}  // namespace esphome

#endif
