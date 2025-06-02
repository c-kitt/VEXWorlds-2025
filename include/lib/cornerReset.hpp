#ifndef CORNER_RESET_HPP
#define CORNER_RESET_HPP

// A single function to corner-reset the robot's position
// given the quadrant (1..4).
//
//   Quadrant 1: top-right
//   Quadrant 2: bottom-right
//   Quadrant 3: bottom-left
//   Quadrant 4: top-left
//
// In this new orientation:
//   - Robot's local X-axis: left -> right (+ right)
//   - Robot's local Y-axis: front -> back (+ front?)
//   - Heading=0° means facing +Y (up), angles increase clockwise.
//
// We'll assume sensors are physically placed left/right of center
// at local X = ± some offset, local Y=0 (or as measured on your robot).

void cornerReset(int quadrant);

#endif // CORNER_RESET_HPP
