/*
    VowelClassifier.cpp

    Original vision pipeline, unmodified. Only two things were added
    at the end of this file: VowelClassifier::begin() and
    VowelClassifier::captureAndClassify(), which wrap the existing
    allocateProcessingBuffers(), initialiseCamera() and classifyFrame()
    so the main sketch does not need to know the internals.
*/

#include "VowelClassifier.h"

#include "esp_heap_caps.h"
#include <math.h>
#include <string.h>

#define CAMERA_MODEL_XIAO_ESP32S3
#include "camera_pins.h"

/* ---------------- Module state ---------------- */

namespace {
bool cameraStatus = false;
uint32_t fileCount = 0;
}

/* ---------------- PSRAM buffers ---------------- */

uint8_t* cropGray = nullptr;
uint8_t* blurImage = nullptr;
uint8_t* binaryImage = nullptr;
uint8_t* tempImage = nullptr;
uint8_t* cleanedImage = nullptr;
uint8_t* candidateImage = nullptr;
uint8_t* candidateTemp = nullptr;

int16_t* labelsCrop = nullptr;
int16_t* labelsCanvas = nullptr;
int16_t* backgroundLabelsCanvas = nullptr;
int32_t* floodQueue = nullptr;

// Integral image used ONLY by the shape-independent blank detector.
// Size is (CROP_WIDTH + 1) x (CROP_HEIGHT + 1).
uint32_t* blankIntegral = nullptr;


// ---------------- Utility ----------------

template <typename T>
T clampValue(T value, T minimum, T maximum) {
  if (value < minimum) return minimum;
  if (value > maximum) return maximum;
  return value;
}

void appendReason(char* destination, size_t capacity, const char* text) {
  size_t used = strlen(destination);
  if (used >= capacity - 1) return;

  if (used > 0) {
    strncat(destination, "; ", capacity - strlen(destination) - 1);
  }

  strncat(destination, text, capacity - strlen(destination) - 1);
}

char selectHighestScore(const Scores& s, float& bestScore) {
  char vowel = 'a';
  bestScore = s.a;

  if (s.e > bestScore) {
    bestScore = s.e;
    vowel = 'e';
  }
  if (s.i > bestScore) {
    bestScore = s.i;
    vowel = 'i';
  }
  if (s.o > bestScore) {
    bestScore = s.o;
    vowel = 'o';
  }
  if (s.u > bestScore) {
    bestScore = s.u;
    vowel = 'u';
  }

  return vowel;
}

bool allocatePsramBuffer(void** pointer, size_t bytes, const char* name) {
  *pointer = heap_caps_malloc(
    bytes,
    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
  );

  if (*pointer == nullptr) {
    Serial.printf("Failed to allocate %s (%u bytes)\n", name, bytes);
    return false;
  }

  Serial.printf("Allocated %-18s %8u bytes in PSRAM\n", name, bytes);
  return true;
}

bool allocateProcessingBuffers() {
  bool ok = true;

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&cropGray),
    CROP_PIXELS,
    "cropGray"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&blurImage),
    CROP_PIXELS,
    "blurImage"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&binaryImage),
    CROP_PIXELS,
    "binaryImage"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&tempImage),
    CROP_PIXELS,
    "tempImage"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&cleanedImage),
    CROP_PIXELS,
    "cleanedImage"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&candidateImage),
    CANVAS_PIXELS,
    "candidateImage"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&candidateTemp),
    CANVAS_PIXELS,
    "candidateTemp"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&labelsCrop),
    CROP_PIXELS * sizeof(int16_t),
    "labelsCrop"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&labelsCanvas),
    CANVAS_PIXELS * sizeof(int16_t),
    "labelsCanvas"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&backgroundLabelsCanvas),
    CANVAS_PIXELS * sizeof(int16_t),
    "backgroundLabelsCanvas"
  );

  const size_t queueElements = max(CROP_PIXELS, CANVAS_PIXELS);

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&floodQueue),
    queueElements * sizeof(int32_t),
    "floodQueue"
  );

  ok &= allocatePsramBuffer(
    reinterpret_cast<void**>(&blankIntegral),
    static_cast<size_t>(CROP_WIDTH + 1) *
      (CROP_HEIGHT + 1) *
      sizeof(uint32_t),
    "blankIntegral"
  );

  Serial.printf(
    "Free PSRAM after allocations: %u bytes\n",
    ESP.getFreePsram()
  );

  return ok;
}

// ---------------- PC image transfer over USB Serial ----------------

/*
  Binary transfer protocol used by the companion Python script:

    IMAGE_START,<imageNumber>,<width>,<height>,<byteCount>\n
    <exactly byteCount raw grayscale bytes>

    IMAGE_END,<imageNumber>\n

  The image bytes are the original 320x240 PIXFORMAT_GRAYSCALE camera
  framebuffer. Each pixel is one uint8_t value (0..255).
*/
bool sendFrameToPC(
  const uint8_t* data,
  size_t length,
  uint32_t imageNumber,
  int width,
  int height
) {
  Serial.printf(
    "IMAGE_START,%lu,%d,%d,%u\n",
    static_cast<unsigned long>(imageNumber),
    width,
    height,
    static_cast<unsigned int>(length)
  );

  // Send in moderate chunks for reliable USB-serial transfer.
  constexpr size_t SERIAL_CHUNK_SIZE = 4096;
  size_t totalWritten = 0;
  const uint32_t transferStartMs = millis();

  while (totalWritten < length) {
    const size_t remaining = length - totalWritten;
    const size_t chunk = min(remaining, SERIAL_CHUNK_SIZE);

    const size_t written = Serial.write(
      data + totalWritten,
      chunk
    );

    if (written == 0) {
      /*
          If no host is draining the USB CDC buffer, Serial.write
          returns 0 indefinitely. The original loop spun here
          forever, which stalled the main loop and stopped programs
          being forwarded to the PSoC. Give up instead.
      */
      if (millis() - transferStartMs >= FRAME_STREAM_TIMEOUT_MS) {
        Serial.println();
        Serial.println("IMAGE_ABORTED");
        return false;
      }

      delay(1);
      continue;
    }

    totalWritten += written;
  }

  // This newline is outside the binary payload and is consumed by Python.
  Serial.write('\n');
  Serial.printf(
    "IMAGE_END,%lu\n",
    static_cast<unsigned long>(imageNumber)
  );
  Serial.flush();

  return totalWritten == length;
}

// ---------------- Camera crop ----------------

void copyFixedCrop(const uint8_t* source) {
  for (int y = 0; y < CROP_HEIGHT; ++y) {
    const int sourceY = CROP_Y_START + y;
    const uint8_t* sourceRow =
      source + sourceY * RAW_WIDTH + CROP_X_START;

    memcpy(
      cropGray + y * CROP_WIDTH,
      sourceRow,
      CROP_WIDTH
    );
  }
}


// ---------------- Blank-image detection ----------------

/*
  SHAPE-INDEPENDENT + LED/GLARE-ROBUST BLANK DETECTION

  The chamber may be circular, square, larger, smaller, or differently lit.

  We therefore DO NOT classify the chamber geometry itself.

  Instead:
    1. Compare each blurred pixel against its local 21x21 neighbourhood.
    2. Keep only pixels that are substantially DARKER than that neighbourhood.
    3. Group those dark pixels into connected components.
    4. Require a sufficiently large connected dark component before declaring
       that handwriting is present.

  Bright LED reflections are intentionally ignored:
  a bright reflection can create edges, but the supplied LED-on blank image
  only produced about 23 intensity levels of local dark contrast after blur.
  Actual black handwriting should create much stronger local dark contrast.

  IMPORTANT:
  The existing a/e/i/o/u preprocessing, features, scoring, orientation and
  priority logic below are NOT changed.
*/

static inline uint32_t integralRectangleSum(
  const uint32_t* integral,
  int stride,
  int x1,
  int y1,
  int x2,
  int y2
) {
  const int ax = x1;
  const int ay = y1;
  const int bx = x2 + 1;
  const int by = y2 + 1;

  return
    integral[by * stride + bx]
    - integral[ay * stride + bx]
    - integral[by * stride + ax]
    + integral[ay * stride + ax];
}

bool isBlankCrop(
  const uint8_t* image,
  int width,
  int height,
  int& strongDarkPixels,
  int& largestDarkComponent,
  float& strongDarkRatio,
  float& maximumLocalDarkness
) {
  if (
    image == nullptr ||
    blankIntegral == nullptr ||
    tempImage == nullptr ||
    labelsCrop == nullptr ||
    floodQueue == nullptr ||
    width <= 0 ||
    height <= 0
  ) {
    strongDarkPixels = 0;
    largestDarkComponent = 0;
    strongDarkRatio = 0.0f;
    maximumLocalDarkness = 0.0f;
    return true;
  }

  const int integralWidth = width + 1;
  const int integralHeight = height + 1;
  const int pixels = width * height;

  memset(
    blankIntegral,
    0,
    static_cast<size_t>(integralWidth) *
      integralHeight *
      sizeof(uint32_t)
  );

  // tempImage is used only as the temporary blank-detector mask here.
  memset(tempImage, 0, static_cast<size_t>(pixels));

  // Build summed-area table.
  for (int y = 0; y < height; ++y) {
    uint32_t rowSum = 0;

    for (int x = 0; x < width; ++x) {
      rowSum += image[y * width + x];

      blankIntegral[
        (y + 1) * integralWidth + (x + 1)
      ] =
        blankIntegral[
          y * integralWidth + (x + 1)
        ] +
        rowSum;
    }
  }

  constexpr int LOCAL_RADIUS = 10;  // 21x21 neighbourhood

  /*
    Increased from 18 to 25.

    For the supplied LED-on blank image, the strongest local-dark response
    after the same 5x5 blur is below 25, so the bright glare/ring is rejected.

    Black pen strokes should normally be substantially darker than 25 levels
    below their surrounding surface.
  */
  constexpr float MIN_LOCAL_DARKNESS = 25.0f;

  strongDarkPixels = 0;
  maximumLocalDarkness = 0.0f;

  for (int y = 0; y < height; ++y) {
    const int y1 = max(0, y - LOCAL_RADIUS);
    const int y2 = min(height - 1, y + LOCAL_RADIUS);

    for (int x = 0; x < width; ++x) {
      const int x1 = max(0, x - LOCAL_RADIUS);
      const int x2 = min(width - 1, x + LOCAL_RADIUS);

      const int neighbourhoodPixels =
        (x2 - x1 + 1) *
        (y2 - y1 + 1);

      const uint32_t neighbourhoodSum =
        integralRectangleSum(
          blankIntegral,
          integralWidth,
          x1,
          y1,
          x2,
          y2
        );

      const float localMean =
        static_cast<float>(neighbourhoodSum) /
        max(neighbourhoodPixels, 1);

      const float localDarkness =
        localMean - image[y * width + x];

      maximumLocalDarkness =
        max(maximumLocalDarkness, localDarkness);

      if (localDarkness >= MIN_LOCAL_DARKNESS) {
        tempImage[y * width + x] = 255;
        ++strongDarkPixels;
      }
    }
  }

  strongDarkRatio =
    static_cast<float>(strongDarkPixels) /
    max(pixels, 1);

  /*
    Connected-component filtering.

    This prevents scattered camera noise or a few reflection-edge pixels from
    being interpreted as handwriting. A real pen stroke should form a
    connected dark structure.
  */
  memset(
    labelsCrop,
    0,
    static_cast<size_t>(pixels) * sizeof(int16_t)
  );

  largestDarkComponent = 0;
  int16_t nextLabel = 1;

  for (int startPixel = 0; startPixel < pixels; ++startPixel) {
    if (
      tempImage[startPixel] == 0 ||
      labelsCrop[startPixel] != 0
    ) {
      continue;
    }

    int queueHead = 0;
    int queueTail = 0;
    int componentArea = 0;

    floodQueue[queueTail++] = startPixel;
    labelsCrop[startPixel] = nextLabel;

    while (queueHead < queueTail) {
      const int index = floodQueue[queueHead++];
      const int y = index / width;
      const int x = index - y * width;

      ++componentArea;

      // 8-connected neighbourhood.
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }

          const int xx = x + dx;
          const int yy = y + dy;

          if (
            xx < 0 || xx >= width ||
            yy < 0 || yy >= height
          ) {
            continue;
          }

          const int neighbour = yy * width + xx;

          if (
            tempImage[neighbour] == 0 ||
            labelsCrop[neighbour] != 0
          ) {
            continue;
          }

          labelsCrop[neighbour] = nextLabel;
          floodQueue[queueTail++] = neighbour;
        }
      }
    }

    largestDarkComponent =
      max(largestDarkComponent, componentArea);

    ++nextLabel;
  }

  /*
    A frame is allowed into the existing vowel classifier only when there is
    a coherent dark stroke.

    These values are intentionally conservative so a thin lowercase i is not
    rejected:
      - at least 35 strong-dark pixels overall
      - at least one connected component of 20 pixels
  */
  constexpr int MIN_STRONG_DARK_PIXELS_FOR_VOWEL = 35;
  constexpr int MIN_CONNECTED_DARK_AREA_FOR_VOWEL = 20;

  const bool meaningfulInk =
    strongDarkPixels >= MIN_STRONG_DARK_PIXELS_FOR_VOWEL &&
    largestDarkComponent >= MIN_CONNECTED_DARK_AREA_FOR_VOWEL;

  return !meaningfulInk;
}

void setBlankResult(
  CandidateResult& result,
  const char* reasonText
) {
  memset(&result, 0, sizeof(result));

  result.blank = true;
  result.valid = true;
  result.vowel = ' ';
  result.score = 0.0f;

  strncpy(
    result.reason,
    reasonText,
    sizeof(result.reason) - 1
  );

  result.reason[sizeof(result.reason) - 1] = '\0';
}

void printBlankClassification(
  const CandidateResult& result,
  unsigned long elapsedMs
) {
  Serial.println();
  Serial.println("Final classification");
  Serial.println("--------------------");
  Serial.println("Predicted class: blank");
  Serial.printf(
    "Decision reason: %s\n",
    result.reason
  );
  Serial.printf(
    "Classification time: %lu ms\n",
    elapsedMs
  );
}

// ---------------- 5x5 Gaussian blur ----------------

int reflectedIndex(int value, int length) {
  // Approximate OpenCV BORDER_REFLECT_101.
  if (value < 0) {
    return -value;
  }

  if (value >= length) {
    return 2 * length - value - 2;
  }

  return value;
}

void gaussianBlur5x5(
  const uint8_t* input,
  uint8_t* output,
  int width,
  int height
) {
  // Separable [1,4,6,4,1] kernel; total divisor = 256.
  static const int kernel[5] = {1, 4, 6, 4, 1};

  // Horizontal intermediate uses uint16 values scaled by 16.
  uint16_t* horizontal = static_cast<uint16_t*>(
    heap_caps_malloc(
      static_cast<size_t>(width) * height * sizeof(uint16_t),
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT
    )
  );

  if (horizontal == nullptr) {
    Serial.println("Gaussian horizontal buffer allocation failed");
    memcpy(output, input, static_cast<size_t>(width) * height);
    return;
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int sum = 0;

      for (int k = -2; k <= 2; ++k) {
        int xx = reflectedIndex(x + k, width);
        sum += kernel[k + 2] * input[y * width + xx];
      }

      horizontal[y * width + x] = static_cast<uint16_t>(sum);
    }
  }

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int sum = 0;

      for (int k = -2; k <= 2; ++k) {
        int yy = reflectedIndex(y + k, height);
        sum += kernel[k + 2] * horizontal[yy * width + x];
      }

      output[y * width + x] =
        static_cast<uint8_t>((sum + 128) >> 8);
    }
  }

  free(horizontal);
}

// ---------------- Otsu threshold ----------------

uint8_t calculateOtsuThreshold(
  const uint8_t* image,
  int pixelCount
) {
  uint32_t histogram[256] = {0};

  for (int i = 0; i < pixelCount; ++i) {
    ++histogram[image[i]];
  }

  uint64_t totalIntensity = 0;

  for (int level = 0; level < 256; ++level) {
    totalIntensity +=
      static_cast<uint64_t>(level) * histogram[level];
  }

  uint32_t backgroundCount = 0;
  uint64_t backgroundIntensity = 0;
  double maximumVariance = -1.0;
  uint8_t bestThreshold = 0;

  for (int threshold = 0; threshold < 256; ++threshold) {
    backgroundCount += histogram[threshold];

    if (backgroundCount == 0) {
      continue;
    }

    uint32_t foregroundCount =
      pixelCount - backgroundCount;

    if (foregroundCount == 0) {
      break;
    }

    backgroundIntensity +=
      static_cast<uint64_t>(threshold) *
      histogram[threshold];

    double backgroundMean =
      static_cast<double>(backgroundIntensity) /
      backgroundCount;

    double foregroundMean =
      static_cast<double>(
        totalIntensity - backgroundIntensity
      ) / foregroundCount;

    double difference = backgroundMean - foregroundMean;

    double variance =
      static_cast<double>(backgroundCount) *
      foregroundCount *
      difference *
      difference;

    if (variance > maximumVariance) {
      maximumVariance = variance;
      bestThreshold = static_cast<uint8_t>(threshold);
    }
  }

  return bestThreshold;
}

int applyInvertedThreshold(
  const uint8_t* grayscale,
  uint8_t* binary,
  int pixelCount,
  uint8_t threshold
) {
  int foregroundCount = 0;

  for (int i = 0; i < pixelCount; ++i) {
    const uint8_t value =
      grayscale[i] <= threshold ? 255 : 0;

    binary[i] = value;

    if (value != 0) {
      ++foregroundCount;
    }
  }

  return foregroundCount;
}

// ---------------- Morphology ----------------

// OpenCV's 3x3 MORPH_ELLIPSE corresponds to a cross-shaped footprint.
bool ellipseKernelPosition(int dx, int dy) {
  return (
    (dx == 0 && abs(dy) <= 1) ||
    (dy == 0 && abs(dx) <= 1)
  );
}

void erodeEllipse3x3(
  const uint8_t* input,
  uint8_t* output,
  int width,
  int height
) {
  memset(output, 0, static_cast<size_t>(width) * height);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      bool keep = true;

      for (int dy = -1; dy <= 1 && keep; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!ellipseKernelPosition(dx, dy)) {
            continue;
          }

          const int xx = x + dx;
          const int yy = y + dy;

          if (
            xx < 0 || xx >= width ||
            yy < 0 || yy >= height ||
            input[yy * width + xx] == 0
          ) {
            keep = false;
            break;
          }
        }
      }

      output[y * width + x] = keep ? 255 : 0;
    }
  }
}

void dilateEllipse3x3(
  const uint8_t* input,
  uint8_t* output,
  int width,
  int height
) {
  memset(output, 0, static_cast<size_t>(width) * height);

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      bool found = false;

      for (int dy = -1; dy <= 1 && !found; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (!ellipseKernelPosition(dx, dy)) {
            continue;
          }

          const int xx = x + dx;
          const int yy = y + dy;

          if (
            xx >= 0 && xx < width &&
            yy >= 0 && yy < height &&
            input[yy * width + xx] != 0
          ) {
            found = true;
            break;
          }
        }
      }

      output[y * width + x] = found ? 255 : 0;
    }
  }
}

void openingAndClosing(
  uint8_t* binary,
  uint8_t* temporary,
  int width,
  int height
) {
  /*
    Preserve a small detached i dot.

    The original opening began with erosion, which could erase the dot before
    connected-component analysis. Closing still fills small gaps in strokes.
  */
  dilateEllipse3x3(binary, temporary, width, height);
  erodeEllipse3x3(temporary, binary, width, height);
}

int applyCircularMask(
  uint8_t* image,
  int width,
  int height
) {
  const int centreX = width / 2;
  const int centreY = height / 2;
  const int radius =
    static_cast<int>(min(width, height) * 0.47f);
  const int radiusSquared = radius * radius;

  int foregroundCount = 0;

  for (int y = 0; y < height; ++y) {
    const int dy = y - centreY;

    for (int x = 0; x < width; ++x) {
      const int dx = x - centreX;

      if (dx * dx + dy * dy > radiusSquared) {
        image[y * width + x] = 0;
      }

      if (image[y * width + x] != 0) {
        ++foregroundCount;
      }
    }
  }

  return foregroundCount;
}

// ---------------- Connected components ----------------

int labelConnectedComponents(
  const uint8_t* binary,
  int width,
  int height,
  int16_t* labels,
  bool foregroundMode,
  ComponentStats* components,
  int maxComponents
) {
  const int pixels = width * height;
  memset(labels, 0, static_cast<size_t>(pixels) * sizeof(int16_t));

  int componentCount = 0;
  int16_t nextLabel = 1;

  for (int start = 0; start < pixels; ++start) {
    const bool active = foregroundMode
      ? binary[start] != 0
      : binary[start] == 0;

    if (!active || labels[start] != 0) {
      continue;
    }

    if (componentCount >= maxComponents) {
      break;
    }

    int queueHead = 0;
    int queueTail = 0;

    floodQueue[queueTail++] = start;
    labels[start] = nextLabel;

    int32_t area = 0;
    int64_t sumX = 0;
    int64_t sumY = 0;

    int minX = width;
    int maxX = -1;
    int minY = height;
    int maxY = -1;

    while (queueHead < queueTail) {
      const int index = floodQueue[queueHead++];
      const int y = index / width;
      const int x = index - y * width;

      ++area;
      sumX += x;
      sumY += y;

      minX = min(minX, x);
      maxX = max(maxX, x);
      minY = min(minY, y);
      maxY = max(maxY, y);

      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) {
          if (dx == 0 && dy == 0) {
            continue;
          }

          const int xx = x + dx;
          const int yy = y + dy;

          if (
            xx < 0 || xx >= width ||
            yy < 0 || yy >= height
          ) {
            continue;
          }

          const int neighbour = yy * width + xx;

          if (labels[neighbour] != 0) {
            continue;
          }

          const bool neighbourActive = foregroundMode
            ? binary[neighbour] != 0
            : binary[neighbour] == 0;

          if (!neighbourActive) {
            continue;
          }

          labels[neighbour] = nextLabel;
          floodQueue[queueTail++] = neighbour;
        }
      }
    }

    ComponentStats& component = components[componentCount];

    component.label = nextLabel;
    component.area = area;
    component.minX = minX;
    component.maxX = maxX;
    component.minY = minY;
    component.maxY = maxY;
    component.centreX =
      area > 0 ? static_cast<float>(sumX) / area : 0.0f;
    component.centreY =
      area > 0 ? static_cast<float>(sumY) / area : 0.0f;

    ++componentCount;
    ++nextLabel;
  }

  return componentCount;
}

void sortComponentsByArea(
  ComponentStats* components,
  int count
) {
  for (int i = 0; i < count - 1; ++i) {
    for (int j = i + 1; j < count; ++j) {
      if (components[j].area > components[i].area) {
        ComponentStats temporary = components[i];
        components[i] = components[j];
        components[j] = temporary;
      }
    }
  }
}

float componentPrincipalAngle(
  const int16_t* labels,
  int width,
  int height,
  int16_t targetLabel,
  float& elongation,
  float& shortSide,
  float& longSide
) {
  int64_t count = 0;
  double sumX = 0.0;
  double sumY = 0.0;
  double sumXX = 0.0;
  double sumYY = 0.0;
  double sumXY = 0.0;

  int minX = width;
  int maxX = -1;
  int minY = height;
  int maxY = -1;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (labels[y * width + x] != targetLabel) {
        continue;
      }

      ++count;
      sumX += x;
      sumY += y;
      sumXX += static_cast<double>(x) * x;
      sumYY += static_cast<double>(y) * y;
      sumXY += static_cast<double>(x) * y;

      minX = min(minX, x);
      maxX = max(maxX, x);
      minY = min(minY, y);
      maxY = max(maxY, y);
    }
  }

  if (count < 2) {
    elongation = 1.0f;
    shortSide = 1.0f;
    longSide = 1.0f;
    return 0.0f;
  }

  const double meanX = sumX / count;
  const double meanY = sumY / count;
  const double varianceX = sumXX / count - meanX * meanX;
  const double varianceY = sumYY / count - meanY * meanY;
  const double covariance = sumXY / count - meanX * meanY;

  const float angle =
    0.5f * atan2f(
      static_cast<float>(2.0 * covariance),
      static_cast<float>(varianceX - varianceY)
    );

  // Use covariance eigenvalues for an orientation-independent elongation.
  const double trace = varianceX + varianceY;
  const double discriminant = sqrt(
    max(
      0.0,
      (varianceX - varianceY) * (varianceX - varianceY) +
      4.0 * covariance * covariance
    )
  );

  const double lambda1 = max(0.0, (trace + discriminant) * 0.5);
  const double lambda2 = max(0.0, (trace - discriminant) * 0.5);

  longSide = static_cast<float>(4.0 * sqrt(lambda1 + 1e-9));
  shortSide = static_cast<float>(4.0 * sqrt(lambda2 + 1e-9));
  elongation = longSide / max(shortSide, 1.0f);

  return angle;
}

int keepRelevantComponents(
  const uint8_t* binary,
  uint8_t* cleaned,
  int width,
  int height,
  int16_t* labels
) {
  constexpr int MAX_COMPONENTS = 64;
  constexpr int MIN_KEPT_COMPONENT_AREA = 8;

  // Static storage avoids increasing the ESP32 task stack.
  static ComponentStats components[MAX_COMPONENTS];

  int count = labelConnectedComponents(
    binary,
    width,
    height,
    labels,
    true,
    components,
    MAX_COMPONENTS
  );

  int validCount = 0;

  for (int i = 0; i < count; ++i) {
    if (components[i].area >= MIN_KEPT_COMPONENT_AREA) {
      components[validCount++] = components[i];
    }
  }

  if (validCount == 0) {
    memset(cleaned, 0, static_cast<size_t>(width) * height);
    return 0;
  }

  sortComponentsByArea(components, validCount);

  const ComponentStats& mainComponent = components[0];
  int16_t keptDotLabel = 0;

  float elongation = 1.0f;
  float shortSide = 1.0f;
  float longSide = 1.0f;

  const float axisAngle = componentPrincipalAngle(
    labels,
    width,
    height,
    mainComponent.label,
    elongation,
    shortSide,
    longSide
  );

  const float axisX = cosf(axisAngle);
  const float axisY = sinf(axisAngle);
  float bestQuality = -1000000.0f;

  /*
    Preserve one compact component aligned with the main component axis.
    This works when i is slanted because it does not require the dot to be
    vertically above the body in the original camera coordinates.
  */
  if (elongation >= 1.15f) {
    for (int i = 1; i < validCount; ++i) {
      const ComponentStats& component = components[i];

      const float areaRatio =
        static_cast<float>(component.area) /
        max(mainComponent.area, 1L);

      if (areaRatio < 0.006f || areaRatio > 0.45f) {
        continue;
      }

      const int componentWidth =
        component.maxX - component.minX + 1;
      const int componentHeight =
        component.maxY - component.minY + 1;
      const int componentBoxArea =
        max(1, componentWidth * componentHeight);

      const float compactness =
        static_cast<float>(component.area) /
        componentBoxArea;

      const float componentAspect =
        static_cast<float>(componentWidth) /
        max(componentHeight, 1);

      if (
        compactness < 0.20f ||
        componentAspect < 0.22f ||
        componentAspect > 4.50f
      ) {
        continue;
      }

      const float dx =
        component.centreX - mainComponent.centreX;
      const float dy =
        component.centreY - mainComponent.centreY;
      const float distance = sqrtf(dx * dx + dy * dy);

      if (distance < 2.0f) {
        continue;
      }

      const float axialDistance =
        fabsf(dx * axisX + dy * axisY);
      const float perpendicularDistance =
        fabsf(dx * axisY - dy * axisX);

      const bool closeToAxis =
        perpendicularDistance <= max(9.0f, shortSide * 3.30f);

      const bool separatedAlongAxis =
        axialDistance >= max(3.0f, shortSide * 0.30f) &&
        axialDistance <= max(24.0f, longSide * 2.10f);

      if (!closeToAxis || !separatedAlongAxis) {
        continue;
      }

      const float quality =
        axialDistance / max(longSide, 1.0f) * 2.0f -
        perpendicularDistance / max(shortSide, 1.0f) +
        compactness -
        fabsf(areaRatio - 0.10f);

      if (quality > bestQuality) {
        bestQuality = quality;
        keptDotLabel = component.label;
      }
    }
  }

  int foregroundCount = 0;
  const int pixels = width * height;

  for (int i = 0; i < pixels; ++i) {
    const bool keep =
      labels[i] == mainComponent.label ||
      (keptDotLabel != 0 && labels[i] == keptDotLabel);

    cleaned[i] = keep ? 255 : 0;

    if (keep) {
      ++foregroundCount;
    }
  }

  return foregroundCount;
}

// ---------------- Orientation and rotation ----------------

float estimateMainAxisAngle(
  const uint8_t* binary,
  int width,
  int height
) {
  int64_t count = 0;
  double sumX = 0.0;
  double sumY = 0.0;
  double sumXX = 0.0;
  double sumYY = 0.0;
  double sumXY = 0.0;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (binary[y * width + x] == 0) {
        continue;
      }

      ++count;
      sumX += x;
      sumY += y;
      sumXX += static_cast<double>(x) * x;
      sumYY += static_cast<double>(y) * y;
      sumXY += static_cast<double>(x) * y;
    }
  }

  if (count < 2) {
    return 90.0f;
  }

  const double meanX = sumX / count;
  const double meanY = sumY / count;
  const double varianceX = sumXX / count - meanX * meanX;
  const double varianceY = sumYY / count - meanY * meanY;
  const double covariance = sumXY / count - meanX * meanY;

  const float radians =
    0.5f * atan2f(
      static_cast<float>(2.0 * covariance),
      static_cast<float>(varianceX - varianceY)
    );

  return radians * 180.0f / PI;
}

bool foregroundBounds(
  const uint8_t* image,
  int width,
  int height,
  int& minX,
  int& minY,
  int& maxX,
  int& maxY
) {
  minX = width;
  minY = height;
  maxX = -1;
  maxY = -1;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (image[y * width + x] == 0) {
        continue;
      }

      minX = min(minX, x);
      minY = min(minY, y);
      maxX = max(maxX, x);
      maxY = max(maxY, y);
    }
  }

  return maxX >= minX && maxY >= minY;
}

void createNormalisedCandidate(
  const uint8_t* source,
  int sourceWidth,
  int sourceHeight,
  float angleDegrees,
  uint8_t* output
) {
  memset(output, 0, CANVAS_PIXELS);

  int minX;
  int minY;
  int maxX;
  int maxY;

  if (!foregroundBounds(
    source,
    sourceWidth,
    sourceHeight,
    minX,
    minY,
    maxX,
    maxY
  )) {
    return;
  }

  // Match Python crop_to_foreground(..., padding=2).
  minX = max(0, minX - 2);
  minY = max(0, minY - 2);
  maxX = min(sourceWidth - 1, maxX + 2);
  maxY = min(sourceHeight - 1, maxY + 2);

  const float sourceCentreX = sourceWidth / 2.0f;
  const float sourceCentreY = sourceHeight / 2.0f;

  const float radians = angleDegrees * PI / 180.0f;
  const float cosine = cosf(radians);
  const float sine = sinf(radians);

  // OpenCV getRotationMatrix2D image-coordinate convention:
  // x' =  cos*x + sin*y
  // y' = -sin*x + cos*y
  auto rotatePoint = [&](float x, float y, float& rx, float& ry) {
    const float dx = x - sourceCentreX;
    const float dy = y - sourceCentreY;

    rx = cosine * dx + sine * dy;
    ry = -sine * dx + cosine * dy;
  };

  float cornerX[4] = {
    static_cast<float>(minX),
    static_cast<float>(maxX),
    static_cast<float>(maxX),
    static_cast<float>(minX)
  };

  float cornerY[4] = {
    static_cast<float>(minY),
    static_cast<float>(minY),
    static_cast<float>(maxY),
    static_cast<float>(maxY)
  };

  float rotatedMinX = 1e9f;
  float rotatedMinY = 1e9f;
  float rotatedMaxX = -1e9f;
  float rotatedMaxY = -1e9f;

  for (int i = 0; i < 4; ++i) {
    float rx;
    float ry;

    rotatePoint(cornerX[i], cornerY[i], rx, ry);

    rotatedMinX = min(rotatedMinX, rx);
    rotatedMaxX = max(rotatedMaxX, rx);
    rotatedMinY = min(rotatedMinY, ry);
    rotatedMaxY = max(rotatedMaxY, ry);
  }

  const float rotatedWidth =
    max(1.0f, rotatedMaxX - rotatedMinX + 1.0f);
  const float rotatedHeight =
    max(1.0f, rotatedMaxY - rotatedMinY + 1.0f);

  const float available =
    CANVAS_SIZE - 2.0f * CANVAS_MARGIN;

  const float scale = min(
    available / rotatedWidth,
    available / rotatedHeight
  );

  const int resizedWidth = max(
    1,
    static_cast<int>(roundf(rotatedWidth * scale))
  );

  const int resizedHeight = max(
    1,
    static_cast<int>(roundf(rotatedHeight * scale))
  );

  const int destinationX0 =
    (CANVAS_SIZE - resizedWidth) / 2;

  const int destinationY0 =
    (CANVAS_SIZE - resizedHeight) / 2;

  // Inverse mapping, equivalent in spirit to INTER_NEAREST.
  for (int destinationY = 0;
       destinationY < resizedHeight;
       ++destinationY) {

    for (int destinationX = 0;
         destinationX < resizedWidth;
         ++destinationX) {

      const float rotatedX =
        rotatedMinX +
        (
          (destinationX + 0.5f) *
          rotatedWidth /
          resizedWidth
        ) -
        0.5f;

      const float rotatedY =
        rotatedMinY +
        (
          (destinationY + 0.5f) *
          rotatedHeight /
          resizedHeight
        ) -
        0.5f;

      // Inverse of:
      // x' =  cos*x + sin*y
      // y' = -sin*x + cos*y
      const float sourceDX =
        cosine * rotatedX -
        sine * rotatedY;

      const float sourceDY =
        sine * rotatedX +
        cosine * rotatedY;

      const int sourceX = static_cast<int>(
        roundf(sourceDX + sourceCentreX)
      );

      const int sourceY = static_cast<int>(
        roundf(sourceDY + sourceCentreY)
      );

      if (
        sourceX < 0 || sourceX >= sourceWidth ||
        sourceY < 0 || sourceY >= sourceHeight
      ) {
        continue;
      }

      const int outputX = destinationX0 + destinationX;
      const int outputY = destinationY0 + destinationY;

      output[outputY * CANVAS_SIZE + outputX] =
        source[sourceY * sourceWidth + sourceX];
    }
  }
}

// ---------------- Feature helpers ----------------

float regionDensity(
  const uint8_t* image,
  int width,
  int height,
  float y1Fraction,
  float y2Fraction,
  float x1Fraction,
  float x2Fraction
) {
  const int y1 = clampValue(
    static_cast<int>(height * y1Fraction),
    0,
    height - 1
  );

  const int y2 = clampValue(
    max(y1 + 1, static_cast<int>(height * y2Fraction)),
    1,
    height
  );

  const int x1 = clampValue(
    static_cast<int>(width * x1Fraction),
    0,
    width - 1
  );

  const int x2 = clampValue(
    max(x1 + 1, static_cast<int>(width * x2Fraction)),
    1,
    width
  );

  int foreground = 0;
  int total = 0;

  for (int y = y1; y < y2; ++y) {
    for (int x = x1; x < x2; ++x) {
      ++total;

      if (image[y * width + x] != 0) {
        ++foreground;
      }
    }
  }

  return total > 0
    ? static_cast<float>(foreground) / total
    : 0.0f;
}

int countRowTransitions(
  const uint8_t* image,
  int width,
  int y
) {
  int transitions = 0;
  bool previous = image[y * width] != 0;

  for (int x = 1; x < width; ++x) {
    const bool current = image[y * width + x] != 0;

    if (current != previous) {
      ++transitions;
    }

    previous = current;
  }

  return transitions;
}

int countColumnTransitions(
  const uint8_t* image,
  int width,
  int height,
  int x
) {
  int transitions = 0;
  bool previous = image[x] != 0;

  for (int y = 1; y < height; ++y) {
    const bool current = image[y * width + x] != 0;

    if (current != previous) {
      ++transitions;
    }

    previous = current;
  }

  return transitions;
}

float verticalSymmetry(
  const uint8_t* image,
  int width,
  int height
) {
  int intersection = 0;
  int unionCount = 0;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const bool first = image[y * width + x] != 0;
      const bool second =
        image[y * width + (width - 1 - x)] != 0;

      if (first || second) {
        ++unionCount;
      }

      if (first && second) {
        ++intersection;
      }
    }
  }

  return unionCount > 0
    ? static_cast<float>(intersection) / unionCount
    : 0.0f;
}

float horizontalSymmetry(
  const uint8_t* image,
  int width,
  int height
) {
  int intersection = 0;
  int unionCount = 0;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const bool first = image[y * width + x] != 0;
      const bool second =
        image[(height - 1 - y) * width + x] != 0;

      if (first || second) {
        ++unionCount;
      }

      if (first && second) {
        ++intersection;
      }
    }
  }

  return unionCount > 0
    ? static_cast<float>(intersection) / unionCount
    : 0.0f;
}

float rotation180Symmetry(
  const uint8_t* image,
  int width,
  int height
) {
  int intersection = 0;
  int unionCount = 0;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const bool first = image[y * width + x] != 0;
      const bool second =
        image[
          (height - 1 - y) * width +
          (width - 1 - x)
        ] != 0;

      if (first || second) {
        ++unionCount;
      }

      if (first && second) {
        ++intersection;
      }
    }
  }

  return unionCount > 0
    ? static_cast<float>(intersection) / unionCount
    : 0.0f;
}

float estimateComponentRoundness(
  const uint8_t* image,
  int width,
  int height,
  const int16_t* labels,
  int16_t label
) {
  int area = 0;
  int perimeter = 0;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = y * width + x;

      if (labels[index] != label) {
        continue;
      }

      ++area;

      const int neighbours[4][2] = {
        {-1, 0},
        {1, 0},
        {0, -1},
        {0, 1}
      };

      for (const auto& neighbour : neighbours) {
        const int xx = x + neighbour[0];
        const int yy = y + neighbour[1];

        if (
          xx < 0 || xx >= width ||
          yy < 0 || yy >= height ||
          labels[yy * width + xx] != label
        ) {
          ++perimeter;
        }
      }
    }
  }

  if (perimeter <= 0) {
    return 0.0f;
  }

  return 4.0f * PI * area /
    (static_cast<float>(perimeter) * perimeter);
}

void calculateHoleFeatures(
  const uint8_t* image,
  int width,
  int height,
  int16_t* backgroundLabels,
  int& holeCount,
  float& largestHoleX,
  float& largestHoleY,
  float& largestHoleAreaRatio
) {
  constexpr int MAX_COMPONENTS = 128;
  static ComponentStats backgroundComponents[MAX_COMPONENTS];

  int count = labelConnectedComponents(
    image,
    width,
    height,
    backgroundLabels,
    false,
    backgroundComponents,
    MAX_COMPONENTS
  );

  const int minimumHoleArea =
    max(10, static_cast<int>(width * height * 0.002f));

  holeCount = 0;
  int largestArea = 0;
  largestHoleX = 0.0f;
  largestHoleY = 0.0f;
  largestHoleAreaRatio = 0.0f;

  for (int i = 0; i < count; ++i) {
    const ComponentStats& component =
      backgroundComponents[i];

    const bool touchesBorder =
      component.minX == 0 ||
      component.minY == 0 ||
      component.maxX == width - 1 ||
      component.maxY == height - 1;

    if (
      touchesBorder ||
      component.area < minimumHoleArea
    ) {
      continue;
    }

    ++holeCount;

    if (component.area > largestArea) {
      largestArea = component.area;
      largestHoleX =
        component.centreX / max(width - 1, 1);
      largestHoleY =
        component.centreY / max(height - 1, 1);
      largestHoleAreaRatio =
        static_cast<float>(component.area) /
        (width * height);
    }
  }
}

void calculateApproximateShapeFeatures(
  const uint8_t* image,
  int width,
  int height,
  float& circularity,
  float& solidity
) {
  int area = 0;
  int perimeter = 0;

  int minX = width;
  int minY = height;
  int maxX = -1;
  int maxY = -1;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (image[y * width + x] == 0) {
        continue;
      }

      ++area;
      minX = min(minX, x);
      maxX = max(maxX, x);
      minY = min(minY, y);
      maxY = max(maxY, y);

      if (x == 0 || image[y * width + x - 1] == 0) ++perimeter;
      if (x == width - 1 || image[y * width + x + 1] == 0) ++perimeter;
      if (y == 0 || image[(y - 1) * width + x] == 0) ++perimeter;
      if (y == height - 1 || image[(y + 1) * width + x] == 0) ++perimeter;
    }
  }

  circularity = perimeter > 0
    ? 4.0f * PI * area /
      (static_cast<float>(perimeter) * perimeter)
    : 0.0f;

  const int boundingArea =
    (maxX >= minX && maxY >= minY)
      ? (maxX - minX + 1) * (maxY - minY + 1)
      : 0;

  // This embedded value is a bounding-box compactness approximation.
  // The current Python scoring rules do not use solidity.
  solidity = boundingArea > 0
    ? static_cast<float>(area) / boundingArea
    : 0.0f;
}

float longestMiddleHorizontalRunRatio(
  const uint8_t* image,
  int width,
  int height
) {
  const int yStart = static_cast<int>(height * 0.38f);
  const int yEnd = static_cast<int>(height * 0.62f);

  int bestRun = 0;

  for (int y = yStart; y < yEnd; ++y) {
    int currentRun = 0;

    for (int x = 0; x < width; ++x) {
      if (image[y * width + x] != 0) {
        ++currentRun;
        bestRun = max(bestRun, currentRun);
      } else {
        currentRun = 0;
      }
    }
  }

  return static_cast<float>(bestRun) / max(width, 1);
}

float narrowSideImbalance(
  const uint8_t* image,
  int width,
  int height,
  int minX,
  int minY,
  int maxX,
  int maxY
) {
  const int shapeWidth = max(1, maxX - minX + 1);
  const int shapeHeight = max(1, maxY - minY + 1);
  const int bandWidth = max(2, static_cast<int>(shapeWidth * 0.16f));

  const int leftEnd = min(maxX + 1, minX + bandWidth);
  const int rightStart = max(minX, maxX - bandWidth + 1);

  int leftCount = 0;
  int rightCount = 0;
  int bandPixels = max(1, bandWidth * shapeHeight);

  for (int y = minY; y <= maxY; ++y) {
    for (int x = minX; x < leftEnd; ++x) {
      if (image[y * width + x] != 0) {
        ++leftCount;
      }
    }

    for (int x = rightStart; x <= maxX; ++x) {
      if (image[y * width + x] != 0) {
        ++rightCount;
      }
    }
  }

  return fabsf(
    static_cast<float>(rightCount - leftCount) /
    bandPixels
  );
}

bool extractFeatures(
  const uint8_t* image,
  int width,
  int height,
  int16_t* foregroundLabels,
  int16_t* backgroundLabels,
  Features& features
) {
  constexpr int MAX_COMPONENTS = 64;
  static ComponentStats components[MAX_COMPONENTS];

  int rawCount = labelConnectedComponents(
    image,
    width,
    height,
    foregroundLabels,
    true,
    components,
    MAX_COMPONENTS
  );

  int componentCount = 0;

  for (int i = 0; i < rawCount; ++i) {
    if (components[i].area >= MIN_COMPONENT_AREA) {
      components[componentCount++] = components[i];
    }
  }

  if (componentCount == 0) {
    return false;
  }

  sortComponentsByArea(components, componentCount);

  int minX = width;
  int minY = height;
  int maxX = -1;
  int maxY = -1;
  int foregroundCount = 0;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (image[y * width + x] == 0) {
        continue;
      }

      ++foregroundCount;
      minX = min(minX, x);
      minY = min(minY, y);
      maxX = max(maxX, x);
      maxY = max(maxY, y);
    }
  }

  features.component_count = componentCount;
  features.width = maxX - minX + 1;
  features.height = maxY - minY + 1;
  features.aspect_ratio =
    static_cast<float>(features.width) /
    max(features.height, 1);

  features.foreground_ratio =
    static_cast<float>(foregroundCount) /
    (width * height);

  calculateApproximateShapeFeatures(
    image,
    width,
    height,
    features.circularity,
    features.solidity
  );

  features.top_density =
    regionDensity(image, width, height, 0.00f, 0.33f, 0.00f, 1.00f);
  features.middle_density =
    regionDensity(image, width, height, 0.33f, 0.67f, 0.00f, 1.00f);
  features.bottom_density =
    regionDensity(image, width, height, 0.67f, 1.00f, 0.00f, 1.00f);

  features.left_density =
    regionDensity(image, width, height, 0.00f, 1.00f, 0.00f, 0.33f);
  features.centre_density =
    regionDensity(image, width, height, 0.00f, 1.00f, 0.33f, 0.67f);
  features.right_density =
    regionDensity(image, width, height, 0.00f, 1.00f, 0.67f, 1.00f);

  features.top_centre_density =
    regionDensity(image, width, height, 0.00f, 0.33f, 0.33f, 0.67f);
  features.bottom_centre_density =
    regionDensity(image, width, height, 0.67f, 1.00f, 0.33f, 0.67f);
  features.middle_left_density =
    regionDensity(image, width, height, 0.33f, 0.67f, 0.00f, 0.50f);
  features.middle_right_density =
    regionDensity(image, width, height, 0.33f, 0.67f, 0.50f, 1.00f);

  features.centre_row_transitions =
    countRowTransitions(image, width, height / 2);

  features.centre_column_transitions =
    countColumnTransitions(image, width, height, width / 2);

  features.vertical_symmetry =
    verticalSymmetry(image, width, height);

  features.horizontal_symmetry =
    horizontalSymmetry(image, width, height);

  features.rotation_180_symmetry =
    rotation180Symmetry(image, width, height);

  features.middle_horizontal_run_ratio =
    longestMiddleHorizontalRunRatio(image, width, height);

  features.narrow_side_imbalance =
    narrowSideImbalance(
      image,
      width,
      height,
      minX,
      minY,
      maxX,
      maxY
    );

  // IMPORTANT:
  // Perform dot detection while foregroundLabels still contains the
  // foreground connected-component labels.
  features.dot_present = false;
  features.dot_above_body = false;

  if (componentCount >= 2) {
    const ComponentStats& mainComponent = components[0];

    float elongation = 1.0f;
    float shortSide = 1.0f;
    float longSide = 1.0f;

    const float axisAngle = componentPrincipalAngle(
      foregroundLabels,
      width,
      height,
      mainComponent.label,
      elongation,
      shortSide,
      longSide
    );

    const float axisX = cosf(axisAngle);
    const float axisY = sinf(axisAngle);
    float bestDotQuality = -1000000.0f;

    for (int i = 1; i < componentCount; ++i) {
      const ComponentStats& dotComponent = components[i];

      const float areaRatio =
        static_cast<float>(dotComponent.area) /
        max(mainComponent.area, 1L);

      const int dotWidth =
        dotComponent.maxX - dotComponent.minX + 1;
      const int dotHeight =
        dotComponent.maxY - dotComponent.minY + 1;
      const int dotBoxArea = max(1, dotWidth * dotHeight);

      const float dotCompactness =
        static_cast<float>(dotComponent.area) /
        dotBoxArea;

      const float dotAspect =
        static_cast<float>(dotWidth) /
        max(dotHeight, 1);

      const float dx =
        dotComponent.centreX - mainComponent.centreX;
      const float dy =
        dotComponent.centreY - mainComponent.centreY;

      const float axialDistance =
        fabsf(dx * axisX + dy * axisY);
      const float perpendicularDistance =
        fabsf(dx * axisY - dy * axisX);

      const float dotRoundness =
        estimateComponentRoundness(
          image,
          width,
          height,
          foregroundLabels,
          dotComponent.label
        );

      const bool plausibleSize =
        areaRatio >= 0.006f &&
        areaRatio <= 0.45f;

      const bool compactEnough =
        dotRoundness >= 0.10f ||
        (
          dotCompactness >= 0.22f &&
          dotAspect >= 0.22f &&
          dotAspect <= 4.50f
        );

      const bool alignedWithStem =
        perpendicularDistance <=
          max(10.0f, shortSide * 3.40f);

      const bool separatedFromStem =
        axialDistance >=
          max(3.0f, shortSide * 0.30f) &&
        axialDistance <=
          max(24.0f, longSide * 2.10f);

      const bool stemLike =
        elongation >= 1.15f ||
        features.aspect_ratio <= 0.70f;

      /*
        dot_above_body is still evaluated in the candidate orientation.
        Among the four candidates, only the upright one places the dot above.
      */
      const bool aboveBody =
        dotComponent.centreY < mainComponent.centreY;

      if (
        !plausibleSize ||
        !compactEnough ||
        !alignedWithStem ||
        !separatedFromStem ||
        !stemLike ||
        !aboveBody
      ) {
        continue;
      }

      const float quality =
        axialDistance / max(longSide, 1.0f) * 2.0f -
        perpendicularDistance / max(shortSide, 1.0f) +
        dotCompactness +
        min(dotRoundness, 1.0f);

      if (quality > bestDotQuality) {
        bestDotQuality = quality;
        features.dot_present = true;
        features.dot_above_body = true;
      }
    }
  }

  // Use a separate label map for black-background components and holes.
  calculateHoleFeatures(
    image,
    width,
    height,
    backgroundLabels,
    features.hole_count,
    features.largest_hole_x,
    features.largest_hole_y,
    features.largest_hole_area_ratio
  );

  const bool smallPocketFromE =
    features.hole_count == 1 &&
    features.largest_hole_area_ratio > 0.0f &&
    features.largest_hole_area_ratio <= 0.080f &&
    features.middle_density >
      features.top_density * 1.22f &&
    features.middle_density >
      features.bottom_density * 1.16f &&
    features.middle_horizontal_run_ratio >= 0.16f &&
    features.rotation_180_symmetry < 0.72f;

  if (smallPocketFromE) {
    features.hole_count = 0;
  }

  return true;
}

// ---------------- Scoring ----------------

Scores scoreVowels(
  const Features& f,
  char* reason,
  size_t reasonCapacity
) {
  Scores s = {0, 0, 0, 0, 0};
  reason[0] = '\0';

  const bool verifiedI =
    f.component_count >= 2 &&
    f.dot_present &&
    f.dot_above_body &&
    f.hole_count == 0;

  if (verifiedI) {
    s.i += 14.0f;
  } else {
    s.i -= 8.0f;
  }

  // o
  if (f.hole_count == 1) s.o += 5.0f;
  else s.o -= 5.0f;

  if (f.aspect_ratio >= 0.68f && f.aspect_ratio <= 1.32f) {
    s.o += 2.0f;
  }

  if (f.vertical_symmetry >= 0.72f) s.o += 4.0f;
  if (f.horizontal_symmetry >= 0.68f) s.o += 3.0f;
  if (f.rotation_180_symmetry >= 0.76f) s.o += 4.0f;

  if (
    f.largest_hole_x >= 0.40f &&
    f.largest_hole_x <= 0.60f &&
    f.largest_hole_y >= 0.38f &&
    f.largest_hole_y <= 0.62f
  ) {
    s.o += 3.0f;
  }

  if (
    f.right_density > f.left_density * 1.12f ||
    f.narrow_side_imbalance >= 0.055f
  ) {
    s.o -= 3.0f;
  }

  // a
  if (f.hole_count == 1) s.a += 4.0f;
  else s.a -= 6.0f;

  if (
    f.largest_hole_area_ratio >= 0.14f &&
    f.largest_hole_area_ratio <= 0.24f &&
    f.rotation_180_symmetry < 0.50f
  ) {
    s.a += 12.0f;
  }

  if (
    f.right_density > f.left_density * 1.05f ||
    f.narrow_side_imbalance >= 0.055f
  ) {
    s.a += 5.0f;
  }

  if (f.middle_right_density > f.middle_left_density * 1.12f) {
    s.a += 5.0f;
  }

  if (
    f.right_density > f.left_density &&
    f.largest_hole_x <= 0.52f
  ) {
    s.a += 3.0f;
  }

  if (f.largest_hole_y < 0.58f) s.a += 1.0f;
  if (f.vertical_symmetry < 0.68f) s.a += 1.0f;
  if (f.rotation_180_symmetry < 0.68f) s.a += 2.0f;
  if (f.bottom_density >= f.top_density) s.a += 1.0f;

  if (
    f.vertical_symmetry > 0.78f &&
    f.horizontal_symmetry > 0.72f &&
    f.rotation_180_symmetry > 0.80f
  ) {
    s.a -= 4.0f;
  }

  // e
  const bool noValidDot = !(
    f.component_count >= 2 &&
    f.dot_present &&
    f.dot_above_body
  );

  const bool uLikeStructure =
    f.hole_count == 0 &&
    f.bottom_density > f.top_density * 1.01f &&
    f.bottom_centre_density >
      f.top_centre_density * 1.10f &&
    f.left_density > 0.065f &&
    f.right_density > 0.065f;

  const bool strongMiddleBar =
    f.middle_density > f.top_density * 1.08f &&
    f.middle_density > f.bottom_density * 1.04f &&
    f.centre_row_transitions >= 2 &&
    f.middle_horizontal_run_ratio >= 0.16f &&
    !uLikeStructure;

  const bool veryStrongMiddleBar =
    f.middle_density > f.top_density * 1.16f &&
    f.middle_density > f.bottom_density * 1.10f &&
    f.centre_row_transitions >= 4 &&
    f.middle_horizontal_run_ratio >= 0.20f &&
    !uLikeStructure;

  if (noValidDot && veryStrongMiddleBar) {
    s.e += 12.0f;
  } else if (noValidDot && strongMiddleBar) {
    s.e += 8.0f;
  }

  if (f.hole_count == 0) {
    s.e += 2.0f;
  } else if (
    f.hole_count == 1 &&
    f.largest_hole_area_ratio < 0.14f
  ) {
    s.e += 2.0f;
  }

  if (
    noValidDot &&
    f.middle_left_density >
      f.middle_right_density * 1.08f
  ) {
    s.e += 2.0f;
  }

  if (
    noValidDot &&
    f.left_density > f.right_density * 1.06f
  ) {
    s.e += 2.0f;
  }

  if (noValidDot && f.vertical_symmetry < 0.66f) {
    s.e += 1.0f;
  }

  if (noValidDot && f.rotation_180_symmetry < 0.66f) {
    s.e += 1.0f;
  }

  if (!noValidDot) s.e -= 12.0f;
  if (uLikeStructure) s.e -= 14.0f;

  if (f.aspect_ratio > 2.2f || f.aspect_ratio < 0.32f) {
    s.e -= 8.0f;
  }

  if (
    f.hole_count == 1 &&
    f.largest_hole_area_ratio > 0.09f &&
    f.rotation_180_symmetry > 0.72f
  ) {
    s.e -= 5.0f;
  }

  if (
    f.hole_count == 1 &&
    f.largest_hole_area_ratio >= 0.14f &&
    f.rotation_180_symmetry < 0.50f
  ) {
    s.e -= 12.0f;
  }

  // u
  if (f.hole_count == 0) s.u += 4.0f;
  else s.u -= 5.0f;

  if (f.bottom_density > f.top_density * 1.01f) {
    s.u += 4.0f;
  }

  if (
    f.bottom_centre_density >
      f.top_centre_density * 1.10f
  ) {
    s.u += 5.0f;
  }

  if (
    f.left_density > 0.065f &&
    f.right_density > 0.065f
  ) {
    s.u += 2.0f;
  }

  if (f.vertical_symmetry > 0.48f) {
    s.u += 1.0f;
  }

  float bestScore = 0.0f;
  const char predicted = selectHighestScore(s, bestScore);

  // Build the selected-vowel reason using the same conditions.
  if (predicted == 'i') {
    if (verifiedI) {
      appendReason(
        reason,
        reasonCapacity,
        "+14: verified dot above an elongated stem"
      );
    } else {
      appendReason(
        reason,
        reasonCapacity,
        "-8: no verified upright dot-stem pair"
      );
    }
  }

  if (predicted == 'o') {
    if (f.hole_count == 1)
      appendReason(reason, reasonCapacity, "+5: exactly one enclosed hole");
    if (f.aspect_ratio >= 0.68f && f.aspect_ratio <= 1.32f)
      appendReason(reason, reasonCapacity, "+2: balanced width and height");
    if (f.vertical_symmetry >= 0.72f)
      appendReason(reason, reasonCapacity, "+4: strong vertical symmetry");
    if (f.horizontal_symmetry >= 0.68f)
      appendReason(reason, reasonCapacity, "+3: strong horizontal symmetry");
    if (f.rotation_180_symmetry >= 0.76f)
      appendReason(reason, reasonCapacity, "+4: strong 180-degree symmetry");
    if (
      f.largest_hole_x >= 0.40f &&
      f.largest_hole_x <= 0.60f &&
      f.largest_hole_y >= 0.38f &&
      f.largest_hole_y <= 0.62f
    )
      appendReason(reason, reasonCapacity, "+3: hole is close to the centre");
  }

  if (predicted == 'a') {
    if (f.hole_count == 1)
      appendReason(reason, reasonCapacity, "+4: one enclosed counter");
    if (
      f.largest_hole_area_ratio >= 0.14f &&
      f.largest_hole_area_ratio <= 0.24f &&
      f.rotation_180_symmetry < 0.50f
    )
      appendReason(
        reason,
        reasonCapacity,
        "+12: large enclosed counter with low rotational symmetry"
      );
    if (f.right_density > f.left_density * 1.05f)
      appendReason(reason, reasonCapacity, "+5: denser right-side stem");
    if (f.middle_right_density > f.middle_left_density * 1.12f)
      appendReason(reason, reasonCapacity, "+5: stronger middle-right stem/tail");
    if (
      f.right_density > f.left_density &&
      f.largest_hole_x <= 0.52f
    )
      appendReason(reason, reasonCapacity, "+3: hole lies left of the right stem");
  }

  if (predicted == 'e') {
    if (noValidDot && veryStrongMiddleBar)
      appendReason(
        reason,
        reasonCapacity,
        "+12: very strong middle bar with no verified dot"
      );
    else if (noValidDot && strongMiddleBar)
      appendReason(
        reason,
        reasonCapacity,
        "+8: strong middle bar with no verified dot"
      );

    if (
      f.hole_count == 1 &&
      f.largest_hole_area_ratio < 0.14f
    )
      appendReason(
        reason,
        reasonCapacity,
        "+2: small enclosed or nearly enclosed pocket"
      );

    if (
      noValidDot &&
      f.middle_left_density >
        f.middle_right_density * 1.08f
    )
      appendReason(
        reason,
        reasonCapacity,
        "+2: backup evidence: middle is denser on the left"
      );

    if (
      noValidDot &&
      f.left_density >
        f.right_density * 1.06f
    )
      appendReason(
        reason,
        reasonCapacity,
        "+2: backup evidence: opening/asymmetry toward the right"
      );

    if (noValidDot && f.vertical_symmetry < 0.66f)
      appendReason(reason, reasonCapacity, "+1: low vertical symmetry");

    if (noValidDot && f.rotation_180_symmetry < 0.66f)
      appendReason(reason, reasonCapacity, "+1: low rotational symmetry");
  }

  if (predicted == 'u') {
    if (f.hole_count == 0)
      appendReason(reason, reasonCapacity, "+4: no enclosed hole");
    if (f.bottom_density > f.top_density * 1.08f)
      appendReason(reason, reasonCapacity, "+4: bottom denser than top");
    if (
      f.bottom_centre_density >
        f.top_centre_density * 1.15f
    )
      appendReason(
        reason,
        reasonCapacity,
        "+5: bottom centre joined and top centre open"
      );
    if (
      f.left_density > 0.10f &&
      f.right_density > 0.10f
    )
      appendReason(reason, reasonCapacity, "+2: two side strokes");
  }

  return s;
}

// ---------------- Candidate priority ----------------

bool isVerifiedI(const CandidateResult& result) {
  const Features& f = result.features;

  return (
    f.component_count >= 2 &&
    f.dot_present &&
    f.dot_above_body &&
    f.hole_count == 0
  );
}

bool isVerifiedA(const CandidateResult& result) {
  const Features& f = result.features;

  return (
    f.hole_count == 1 &&
    f.largest_hole_area_ratio >= 0.10f &&
    (
      f.narrow_side_imbalance >= 0.055f ||
      f.right_density > f.left_density * 1.08f ||
      f.middle_right_density >
        f.middle_left_density * 1.16f
    ) &&
    (
      f.rotation_180_symmetry < 0.78f ||
      f.vertical_symmetry < 0.78f
    )
  );
}

bool isVerifiedO(const CandidateResult& result) {
  const Features& f = result.features;

  return (
    f.hole_count == 1 &&
    f.largest_hole_area_ratio >= 0.10f &&
    f.aspect_ratio >= 0.65f &&
    f.aspect_ratio <= 1.40f &&
    f.vertical_symmetry >= 0.72f &&
    f.horizontal_symmetry >= 0.64f &&
    f.rotation_180_symmetry >= 0.68f &&
    f.narrow_side_imbalance < 0.055f
  );
}

bool isVerifiedU(const CandidateResult& result) {
  const Features& f = result.features;

  return (
    f.hole_count == 0 &&
    f.bottom_density > f.top_density * 1.01f &&
    f.bottom_centre_density >
      f.top_centre_density * 1.10f &&
    f.left_density > 0.065f &&
    f.right_density > 0.065f &&
    f.middle_horizontal_run_ratio < 0.22f &&
    !(
      f.component_count >= 2 &&
      f.dot_present &&
      f.dot_above_body
    )
  );
}

bool isVerifiedE(const CandidateResult& result) {
  const Features& f = result.features;

  return (
    !(
      f.component_count >= 2 &&
      f.dot_present &&
      f.dot_above_body
    ) &&
    f.middle_density > f.top_density * 1.08f &&
    f.middle_density > f.bottom_density * 1.04f &&
    f.centre_row_transitions >= 2 &&
    f.middle_horizontal_run_ratio >= 0.16f &&
    !(
      f.hole_count == 0 &&
      f.bottom_density > f.top_density * 1.01f &&
      f.bottom_centre_density >
        f.top_centre_density * 1.10f &&
      f.left_density > 0.065f &&
      f.right_density > 0.065f
    )
  );
}

int selectBestCandidate(CandidateResult candidates[4]) {
  int bestIndex = -1;
  float bestScore = -1e9f;

  // 1. A verified dot-stem pair has absolute priority.
  for (int i = 0; i < 4; ++i) {
    if (
      candidates[i].valid &&
      isVerifiedI(candidates[i]) &&
      candidates[i].scores.i > bestScore
    ) {
      bestScore = candidates[i].scores.i;
      bestIndex = i;
    }
  }

  if (bestIndex >= 0) {
    return bestIndex;
  }

  // 2. For closed shapes, compare verified a and o orientations together.
  for (int i = 0; i < 4; ++i) {
    if (
      candidates[i].valid &&
      (isVerifiedA(candidates[i]) ||
       isVerifiedO(candidates[i])) &&
      candidates[i].score > bestScore
    ) {
      bestScore = candidates[i].score;
      bestIndex = i;
    }
  }

  if (bestIndex >= 0) {
    return bestIndex;
  }

  // 3. Prefer an upright u structure before accepting e.
  bestScore = -1e9f;

  for (int i = 0; i < 4; ++i) {
    if (
      candidates[i].valid &&
      isVerifiedU(candidates[i]) &&
      candidates[i].scores.u > bestScore
    ) {
      bestScore = candidates[i].scores.u;
      bestIndex = i;
    }
  }

  if (bestIndex >= 0) {
    return bestIndex;
  }

  // 4. Verified e requires an actual horizontal middle run.
  bestScore = -1e9f;

  for (int i = 0; i < 4; ++i) {
    if (
      candidates[i].valid &&
      isVerifiedE(candidates[i]) &&
      candidates[i].scores.e > bestScore
    ) {
      bestScore = candidates[i].scores.e;
      bestIndex = i;
    }
  }

  if (bestIndex >= 0) {
    return bestIndex;
  }

  // 5. Original generic maximum fallback.
  bestIndex = 0;
  bestScore = candidates[0].score;

  for (int i = 1; i < 4; ++i) {
    if (
      candidates[i].valid &&
      candidates[i].score > bestScore
    ) {
      bestScore = candidates[i].score;
      bestIndex = i;
    }
  }

  return bestIndex;
}

// ---------------- Printing ----------------

void printCandidateSummary(
  const CandidateResult candidates[4]
) {
  Serial.println();
  Serial.println("Orientation candidates");
  Serial.println("----------------------");

  for (int i = 0; i < 4; ++i) {
    const CandidateResult& result = candidates[i];

    Serial.printf(
      "Candidate %d: angle=%.1f°, prediction=%c, best score=%.1f\n",
      i,
      result.angle,
      result.vowel,
      result.score
    );

    Serial.printf(
      "  Scores: a=%.1f, e=%.1f, i=%.1f, o=%.1f, u=%.1f\n",
      result.scores.a,
      result.scores.e,
      result.scores.i,
      result.scores.o,
      result.scores.u
    );
  }
}

void printFeatures(const Features& f) {
  Serial.println();
  Serial.println("Features from selected orientation");
  Serial.println("----------------------------------");

  Serial.printf("component_count             : %d\n", f.component_count);
  Serial.printf("hole_count                  : %d\n", f.hole_count);
  Serial.printf("width                       : %d\n", f.width);
  Serial.printf("height                      : %d\n", f.height);
  Serial.printf("aspect_ratio                : %.4f\n", f.aspect_ratio);
  Serial.printf("foreground_ratio            : %.4f\n", f.foreground_ratio);
  Serial.printf("circularity                 : %.4f\n", f.circularity);
  Serial.printf("solidity                    : %.4f\n", f.solidity);
  Serial.printf("top_density                 : %.4f\n", f.top_density);
  Serial.printf("middle_density              : %.4f\n", f.middle_density);
  Serial.printf("bottom_density              : %.4f\n", f.bottom_density);
  Serial.printf("left_density                : %.4f\n", f.left_density);
  Serial.printf("centre_density              : %.4f\n", f.centre_density);
  Serial.printf("right_density               : %.4f\n", f.right_density);
  Serial.printf("top_centre_density          : %.4f\n", f.top_centre_density);
  Serial.printf("bottom_centre_density       : %.4f\n", f.bottom_centre_density);
  Serial.printf("middle_left_density         : %.4f\n", f.middle_left_density);
  Serial.printf("middle_right_density        : %.4f\n", f.middle_right_density);
  Serial.printf("centre_row_transitions      : %d\n", f.centre_row_transitions);
  Serial.printf("centre_column_transitions   : %d\n", f.centre_column_transitions);
  Serial.printf("dot_present                 : %s\n", f.dot_present ? "True" : "False");
  Serial.printf("dot_above_body              : %s\n", f.dot_above_body ? "True" : "False");
  Serial.printf("vertical_symmetry           : %.4f\n", f.vertical_symmetry);
  Serial.printf("horizontal_symmetry         : %.4f\n", f.horizontal_symmetry);
  Serial.printf("rotation_180_symmetry       : %.4f\n", f.rotation_180_symmetry);
  Serial.printf("largest_hole_x              : %.4f\n", f.largest_hole_x);
  Serial.printf("largest_hole_y              : %.4f\n", f.largest_hole_y);
  Serial.printf("largest_hole_area_ratio     : %.4f\n", f.largest_hole_area_ratio);
}

// ---------------- Classification ----------------

bool classifyFrame(
  const uint8_t* rawFrame,
  CandidateResult& bestResult
) {
  const unsigned long startTime = millis();

  copyFixedCrop(rawFrame);

  /*
    Blur first so isolated camera noise does not count as handwriting.
    This does NOT change the existing vowel algorithm because the same
    blurImage is already used by the original Otsu stage below.
  */
  gaussianBlur5x5(
    cropGray,
    blurImage,
    CROP_WIDTH,
    CROP_HEIGHT
  );

  int blankStrongDarkPixels = 0;
  int blankLargestDarkComponent = 0;
  float blankStrongDarkRatio = 0.0f;
  float blankMaximumLocalDarkness = 0.0f;

  const bool blankCrop = isBlankCrop(
    blurImage,
    CROP_WIDTH,
    CROP_HEIGHT,
    blankStrongDarkPixels,
    blankLargestDarkComponent,
    blankStrongDarkRatio,
    blankMaximumLocalDarkness
  );

  Serial.println();
  Serial.println("Blank detection");
  Serial.println("---------------");
  Serial.printf(
    "Strong local-dark pixels: %d\n",
    blankStrongDarkPixels
  );
  Serial.printf(
    "Largest dark component: %d\n",
    blankLargestDarkComponent
  );
  Serial.printf(
    "Strong local-dark ratio: %.5f\n",
    blankStrongDarkRatio
  );
  Serial.printf(
    "Maximum local darkness: %.2f\n",
    blankMaximumLocalDarkness
  );

  if (blankCrop) {
    Serial.println();
    Serial.println("Image information");
    Serial.println("-----------------");
    Serial.printf("Original shape: (%d, %d)\n", RAW_HEIGHT, RAW_WIDTH);
    Serial.printf("Cropped shape: (%d, %d)\n", CROP_HEIGHT, CROP_WIDTH);

    setBlankResult(
      bestResult,
      "no sufficiently strong connected dark pen-stroke feature detected"
    );

    printBlankClassification(
      bestResult,
      millis() - startTime
    );

    return true;
  }

  const uint8_t threshold = calculateOtsuThreshold(
    blurImage,
    CROP_PIXELS
  );

  int binaryForeground = applyInvertedThreshold(
    blurImage,
    binaryImage,
    CROP_PIXELS,
    threshold
  );

  openingAndClosing(
    binaryImage,
    tempImage,
    CROP_WIDTH,
    CROP_HEIGHT
  );

  binaryForeground = applyCircularMask(
    binaryImage,
    CROP_WIDTH,
    CROP_HEIGHT
  );

  const int cleanedForeground = keepRelevantComponents(
    binaryImage,
    cleanedImage,
    CROP_WIDTH,
    CROP_HEIGHT,
    labelsCrop
  );

  Serial.println();
  Serial.println("Image information");
  Serial.println("-----------------");
  Serial.printf("Original shape: (%d, %d)\n", RAW_HEIGHT, RAW_WIDTH);
  Serial.printf("Cropped shape: (%d, %d)\n", CROP_HEIGHT, CROP_WIDTH);
  Serial.printf("Otsu threshold: %.1f\n", static_cast<float>(threshold));
  Serial.printf("Binary foreground pixels: %d\n", binaryForeground);
  Serial.printf("Cleaned foreground pixels: %d\n", cleanedForeground);

  if (cleanedForeground == 0) {
    setBlankResult(
      bestResult,
      "no foreground component remained after preprocessing"
    );

    printBlankClassification(
      bestResult,
      millis() - startTime
    );

    return true;
  }

  const float mainAxisAngle = estimateMainAxisAngle(
    cleanedImage,
    CROP_WIDTH,
    CROP_HEIGHT
  );

  const float deskewAngle = 90.0f - mainAxisAngle;
  static const float extraAngles[4] = {
    0.0f,
    90.0f,
    180.0f,
    270.0f
  };

  static CandidateResult candidates[4];

  for (int candidateIndex = 0; candidateIndex < 4; ++candidateIndex) {
    CandidateResult& result = candidates[candidateIndex];
    memset(&result, 0, sizeof(result));
    result.blank = false;

    const float totalAngle =
      fmodf(
        deskewAngle + extraAngles[candidateIndex] + 360.0f,
        360.0f
      );

    result.angle = totalAngle;

    createNormalisedCandidate(
      cleanedImage,
      CROP_WIDTH,
      CROP_HEIGHT,
      totalAngle,
      candidateImage
    );

    result.valid = extractFeatures(
      candidateImage,
      CANVAS_SIZE,
      CANVAS_SIZE,
      labelsCanvas,
      backgroundLabelsCanvas,
      result.features
    );

    if (!result.valid) {
      result.vowel = '?';
      result.score = -1e9f;
      continue;
    }

    result.scores = scoreVowels(
      result.features,
      result.reason,
      sizeof(result.reason)
    );

    result.vowel = selectHighestScore(
      result.scores,
      result.score
    );
  }

  printCandidateSummary(candidates);

  const int bestIndex = selectBestCandidate(candidates);
  bestResult = candidates[bestIndex];

  printFeatures(bestResult.features);

  Serial.println();
  Serial.println("Final classification");
  Serial.println("--------------------");
  Serial.printf("Predicted vowel: %c\n", bestResult.vowel);
  Serial.printf(
    "Selected rotation: %.1f degrees\n",
    bestResult.angle
  );
  Serial.printf(
    "Decision score: %.1f\n",
    bestResult.score
  );
  Serial.printf(
    "Decision reason: %s\n",
    bestResult.reason
  );

  Serial.printf(
    "Classification time: %lu ms\n",
    millis() - startTime
  );

  return true;
}

// ---------------- Camera setup ----------------

bool initialiseCamera() {
  camera_config_t config = {};

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;

  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;

  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;

  const esp_err_t error = esp_camera_init(&config);

  if (error != ESP_OK) {
    Serial.printf(
      "Camera initialization failed: 0x%X (%s)\n",
      error,
      esp_err_to_name(error)
    );
    return false;
  }

  return true;
}

/* ================================================================
   Public interface. The pipeline above is unchanged; these two
   functions simply wrap it.
   ================================================================ */

namespace VowelClassifier {

bool begin() {
  Serial.printf("PSRAM detected: %s\n", psramFound() ? "YES" : "NO");

  if (!psramFound()) {
    Serial.println("PSRAM is required. Select Tools > PSRAM > OPI PSRAM.");
    return false;
  }

  Serial.printf("Total PSRAM: %u bytes\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());

  if (!allocateProcessingBuffers()) {
    Serial.println("Processing buffer allocation failed");
    return false;
  }

  cameraStatus = initialiseCamera();
  return cameraStatus;
}

bool isReady() {
  return cameraStatus;
}

bool captureAndClassify(CandidateResult& result) {
  if (!cameraStatus) return false;

  camera_fb_t* frame = esp_camera_fb_get();

  if (frame == nullptr) {
    Serial.println("Camera capture failed");
    return false;
  }

  if (frame->width != RAW_WIDTH || frame->height != RAW_HEIGHT ||
      frame->len < RAW_PIXELS) {
    Serial.printf("Unexpected frame: %ux%u, %u bytes\n",
                  frame->width, frame->height, frame->len);
    esp_camera_fb_return(frame);
    return false;
  }

  const uint32_t currentImageNumber = fileCount;

  if (ENABLE_FRAME_STREAMING) {
    if (!sendFrameToPC(frame->buf, RAW_PIXELS, currentImageNumber,
                       RAW_WIDTH, RAW_HEIGHT)) {
      Serial.printf("Image %lu transfer failed\n",
                    static_cast<unsigned long>(currentImageNumber));
    }
  }

  const bool success = classifyFrame(frame->buf, result);

  // The framebuffer stays valid throughout classification and is
  // returned only after all processing and printing have completed.
  esp_camera_fb_return(frame);

  ++fileCount;

  if (!success) {
    Serial.printf("Classification failed for image %lu\n",
                  static_cast<unsigned long>(currentImageNumber));
  }

  return success;
}

}  // namespace VowelClassifier
