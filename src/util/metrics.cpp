
#include <stdint.h>

// TODO: Clean up Arduino code from this file
#include <Arduino.h>

#include "../util/utils.h"
#include "../util/metrics.h"

void metricsReset(struct metrics* metrics)
{
  metrics->count = 0;
  metrics->sum = 0;
  metrics->average = 0;
  metrics->maximum = 0;
  metrics->minimum = UINT32_MAX;
  return;
}

void metricsStart(struct metrics* metrics)
{
  metrics->current = (uint32_t) micros();
  return;
}

void metricsStop(struct metrics* metrics)
{
  uint32_t time = ((uint32_t) micros()) - metrics->current;
  metrics->count++;
  
  // Handle rollover of sum (which should always rollover faster than counter)
  if (metrics->sum + time < metrics->sum) {
    // TODO: Find a better solution
    //       For now, set sum to a reasonable multiplier of the average so the current
    //       time can continue to be rolled in without dominating the average
    //       See github PR #34 for a short discussion on possible improvements/modifications
    metrics->count = 100;
    metrics->sum = metrics->average * metrics->count;
  }
  
  metrics->sum += time;
  metrics->average = metrics->sum / metrics->count;
  metrics->minimum = min(time, metrics->minimum);
  metrics->maximum = max(time, metrics->maximum);
  return;
}