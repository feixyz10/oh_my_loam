#include "helper.h"

namespace oh_my_loam {

float GetTime(const TPoint& pt) { return pt.time - GetScanId(pt); }

int GetScanId(const TPoint& pt) { return static_cast<int>(pt.time); }

}  // oh_my_loam