
#ifndef __METRICS_UTIL_H__
#define __METRICS_UTIL_H__

#include <stdint.h>

struct metrics {
  uint32_t count;
  uint32_t sum;
  uint32_t current;
  uint32_t average;
  uint32_t maximum;
  uint32_t minimum;
};

// TODO: Doc
void metricsReset(struct metrics* metrics);

// TODO: Doc
void metricsStart(struct metrics* metrics);

// TODO: Doc
void metricsStop(struct metrics* metrics);

#endif /* __METRICS_UTIL_H__ */