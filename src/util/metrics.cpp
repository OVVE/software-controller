/* 
Copyright 2020 LifeMech  Inc

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

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