#pragma once

#include "tracking/base_tracking_worker.h"
#include "tracking/hm_tracking_worker.h"

static std::unique_ptr<BaseTrackingWorker> createTrackingWorker(const TrackingWorkerParams& params) {
  std::unique_ptr<BaseTrackingWorker> tracking_worker;
  tracking_worker = std::unique_ptr<BaseTrackingWorker>(new HmTrackingWorker(params));
  return tracking_worker;
}
