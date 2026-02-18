#include "path.h"
#include "position.h"

// Store in program memory to save RAM
const PROGMEM int16_t SPEED_FIXED = 5000;  // 50.00 in fixed point
const PROGMEM int16_t GRID_SIZE = 5000;    // 50.00 in fixed point  
const PROGMEM int16_t GRID_OFFSET = 2500;  // 25.00 in fixed point

// Optimized distance calculation using fixed point
int16_t fastDistance(const CompactPosition &a, const CompactPosition &b) {
  int32_t dx = (int32_t)a.x - b.x;
  int32_t dy = (int32_t)a.y - b.y;
  
  // Fast integer square root approximation
  int32_t distSq = dx * dx + dy * dy;
  if (distSq == 0) return 0;
  
  int32_t dist = distSq / 100;  // Rough approximation for fixed point
  return (int16_t)dist;
}

void toAbsoluteCoordinates(PathVector &path) {
  int16_t gridSize = pgm_read_word(&GRID_SIZE);
  int16_t gridOffset = pgm_read_word(&GRID_OFFSET);
  
  for (uint8_t i = 0; i < path.size; i++) {
    CompactPosition* pos = &path.points[i];
    
    // Convert to absolute coordinates using fixed point
    pos->x = (pos->x * gridSize) / 100 + gridOffset;
    pos->y = (pos->y * gridSize) / 100 + gridOffset;
  }

  Serial.print(F("size: "));
  Serial.println(path.size);
}

// Simplified interpolation to save memory
void interpolatePath(CompactPosition* result, uint8_t& resultSize, 
                    const CompactPosition &start, const CompactPosition &end, 
                    uint8_t maxPoints) {
  resultSize = 0;
  int16_t d = fastDistance(start, end);
  
  if (d < 200) {  // Less than 2.00 units, skip interpolation
    return;
  }
  
  uint8_t steps = min((uint8_t)(d / 100), (uint8_t)(maxPoints - 1));
  int16_t speed = pgm_read_word(&SPEED_FIXED) / 100;
  
  for (uint8_t n = 1; n <= steps && resultSize < maxPoints; n++) {
    CompactPosition newPos;
    newPos.x = start.x + ((int32_t)n * (end.x - start.x)) / steps;
    newPos.y = start.y + ((int32_t)n * (end.y - start.y)) / steps;
    newPos.theta = (speed_t)speed;
    
    result[resultSize++] = newPos;
  }
}

void generatePath(PathVector &path, PathResult &result) {
  CompactPosition tempPath[MAX_INTERPOLATED_POINTS];
  uint8_t tempSize;

  for (uint8_t i = 0; i < path.size - 1 && result.count < MAX_SEGMENTS; i++) {
    CompactPosition start = path.at(i);
    CompactPosition end = path.at(i + 1);

    // Create path segment
    PathSegment segment;
    segment.shouldFinishAt = 0;
    segment.type = PATH_SEGMENT;
    
    // Interpolate directly into segment
    interpolatePath(segment.data.path, segment.dataSize, start, end, 
                   MAX_INTERPOLATED_POINTS - 1);

    // Add end point
    if (segment.dataSize < MAX_INTERPOLATED_POINTS) {
      end.theta = (i + 1 == path.size - 1) ? 0 : 
                  (speed_t)(pgm_read_word(&SPEED_FIXED) / 100);
      segment.data.path[segment.dataSize++] = end;
    }

    // Check for 180-degree turn (simplified)
    if (i < path.size - 2) {
      CompactPosition next = path.at(i + 2);
      if (abs(next.x - start.x) < 50 && abs(next.y - start.y) < 50) {  // Close enough
        // Mark as stop
        if (segment.dataSize > 0) {
          segment.data.path[segment.dataSize - 1].theta = 0;
        }
        
        result.push_back(segment);
        
        // Add angle segment
        PathSegment angleSegment;
        angleSegment.shouldFinishAt = 0;
        angleSegment.type = ANGLE_SEGMENT;
        angleSegment.dataSize = 1;
        angleSegment.data.angle = 0;  // Simplified angle calculation
        
        if (result.count < MAX_SEGMENTS) {
          result.push_back(angleSegment);
        }
        continue;
      }
    }

    result.push_back(segment);
  }
}
