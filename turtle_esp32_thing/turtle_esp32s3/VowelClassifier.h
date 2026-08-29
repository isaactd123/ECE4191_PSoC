/*
    VowelClassifier.h

    Camera capture and the rule-based a/e/i/o/u classifier.

    The implementation in VowelClassifier.cpp is your original vision
    pipeline, moved verbatim. No thresholds, features, scoring rules
    or verification predicates were changed.
*/

#pragma once

#include <Arduino.h>
#include <esp_camera.h>
#include "TurtleConfig.h"

/* ---------------- Result structures ---------------- */

struct ComponentStats {
  int16_t label;
  int32_t area;
  int16_t minX;
  int16_t maxX;
  int16_t minY;
  int16_t maxY;
  float centreX;
  float centreY;
};

struct Features {
  int component_count;
  int hole_count;

  int width;
  int height;
  float aspect_ratio;

  float foreground_ratio;
  float circularity;
  float solidity;

  float top_density;
  float middle_density;
  float bottom_density;

  float left_density;
  float centre_density;
  float right_density;

  float top_centre_density;
  float bottom_centre_density;
  float middle_left_density;
  float middle_right_density;

  int centre_row_transitions;
  int centre_column_transitions;

  bool dot_present;
  bool dot_above_body;

  float vertical_symmetry;
  float horizontal_symmetry;
  float rotation_180_symmetry;

  float largest_hole_x;
  float largest_hole_y;
  float largest_hole_area_ratio;

  // Internal structural features. These are deliberately not printed so the
  // existing Serial Monitor output format remains unchanged.
  float middle_horizontal_run_ratio;
  float narrow_side_imbalance;
};

struct Scores {
  float a;
  float e;
  float i;
  float o;
  float u;
};

struct CandidateResult {
  float angle;
  char vowel;
  float score;
  Scores scores;
  Features features;
  char reason[700];
  bool valid;

  // True only when the captured crop contains no meaningful visual feature.
  // This is separate from the a/e/i/o/u scoring so the existing vowel
  // algorithm is left unchanged.
  bool blank;
};

/* ---------------- Public interface ---------------- */

namespace VowelClassifier {

/* Allocates the PSRAM buffers and starts the camera. */
bool begin();

bool isReady();

/*
    Captures a frame and classifies it. Returns false if the capture
    or the classification failed, in which case the caller should
    treat the square as blank.
*/
bool captureAndClassify(CandidateResult& result);

}  // namespace VowelClassifier
