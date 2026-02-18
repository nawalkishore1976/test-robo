#ifndef POSITION_H
#define POSITION_H

#include <math.h>

struct Position {
  float x;
  float y;
  float theta;

  int getDegrees() const { return (int)(theta * 180.0f / M_PI); }
  
  Position() : x(0.0f), y(0.0f), theta(0.0f) {}
  Position(float x, float y, float theta) : x(x), y(y), theta(theta) {}

  // equal
  bool equals(const Position &other, bool checkTheta = true) const {
    return this->x == other.x && this->y == other.y &&
           (checkTheta ? this->theta == other.theta : true);
  }

  bool operator==(const Position &other) const { return equals(other); }

  // subtract operator
  Position operator-(const Position &other) const {
    return Position(x - other.x, y - other.y, theta - other.theta);
  }

  // add operator
  Position operator+(const Position &other) const {
    return Position(x + other.x, y + other.y, theta + other.theta);
  }

  // dot product operator
  float operator*(const Position &other) const {
    return this->x * other.x + this->y * other.y;
  }

  // scalar multiply operator
  Position operator*(const float &other) const {
    return Position(x * other, y * other, theta);
  }

  float distance(const Position &other) const {
    float dx = this->x - other.x;
    float dy = this->y - other.y;
    return sqrt(dx * dx + dy * dy);  // Arduino-compatible sqrt
  }

  Position lerp(Position other, float t) const {
    return Position(this->x + (other.x - this->x) * t,
                    this->y + (other.y - this->y) * t, this->theta);
  }

  float angle(Position other) const {
    return atan2(other.y - this->y, other.x - this->x);  // Arduino-compatible atan2
  }
};

#endif // POSITION_H
