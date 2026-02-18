#ifndef PATH_H
#define PATH_H

#include "position.h"
#include <avr/pgmspace.h>

// Drastically reduced for 6KB RAM - only ~1.5KB total usage
#define MAX_PATH_POINTS 15
#define MAX_SEGMENTS 8
#define MAX_INTERPOLATED_POINTS 20

// Use 16-bit fixed point (divide by 100 to get actual value)
typedef int16_t coord_t;
typedef int8_t speed_t;

struct CompactPosition {
  coord_t x, y;    // 4 bytes total
  speed_t theta;   // 1 byte
} __attribute__((packed));

struct PathVector {
  CompactPosition points[MAX_PATH_POINTS];  // 75 bytes
  uint8_t size;
  
  PathVector() : size(0) {}
  
  void push_back(const CompactPosition& pos) {
    if (size < MAX_PATH_POINTS) {
      points[size++] = pos;
    }
  }
  
  CompactPosition& at(uint8_t index) { return points[index]; }
  void pop_back() { if (size > 0) size--; }
  void clear() { size = 0; }
};

enum SegmentType : uint8_t {
  PATH_SEGMENT = 0,
  ANGLE_SEGMENT = 1
};

struct PathSegment {
  uint8_t shouldFinishAt;  // 1 byte instead of float
  SegmentType type : 1;    // 1 bit
  uint8_t dataSize : 7;    // 7 bits for size
  
  // Store data more compactly
  union {
    CompactPosition path[MAX_INTERPOLATED_POINTS];  // 100 bytes max
    int16_t angle;  // 2 bytes
  } data;
} __attribute__((packed));

struct PathResult {
  PathSegment segments[MAX_SEGMENTS];  // ~800 bytes max
  uint8_t count;
  
  PathResult() : count(0) {}
  
  void push_back(const PathSegment& segment) {
    if (count < MAX_SEGMENTS) {
      segments[count++] = segment;
    }
  }
};

void toAbsoluteCoordinates(PathVector &path);
void generatePath(PathVector &path, PathResult &result);

#endif
