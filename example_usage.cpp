// Instead of std::vector
Position pathArray[10] = {
  {0.0f, 0.0f, 0.0f},
  {1.0f, 1.0f, 0.5f},
  // ... more points
};

// Call the function with array pointer and size
robot::follow(pathArray, 10, 2.5f, 5000, 10.0f);