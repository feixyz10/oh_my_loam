#pragma once

#include "extractor.h"

namespace oh_my_loam {

// for VLP-16
class ExtractorVLP16 : public Extractor {
 public:
  ExtractorVLP16() { num_scans_ = 16; }

 private:
  int GetScanID(const common::Point& pt) const override;
};

}  // namespace oh_my_loam