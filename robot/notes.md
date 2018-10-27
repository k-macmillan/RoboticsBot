# Project Notes

## Camera

The main idea is that there will be one Camera node publishing on two (for now) topics:

1. Difference from lane center
2. Points of interest, published if one is encountered in the current frame.
    * RED stop location.
    * BLUE parking lot exit.
    * YELLOW parking lot obstacle.
    * GREEN graph intersection.

We'll also need a standalone HSV tuner program to calibrate our color masks based on lighting, etc.
Unfortunately, TKinter *must* run in the main thread's event loop, so we cannot use a GUI to update
the masks as the robot is running the course.

## Wheels

A wheel controller to listen to Twist messages containing the robot's linear and angular velocity
in its local reference frame, and to set the left and right wheel speeds appropriately.

## Brain

Listens to the Camera node to determine what state the robot is in, and responds accordingly.
* Interpret state based on camera
* Act based on state

## States

* Road
    * Free travel
    * Stopped at "light"
* Parking lot
    * ??
* Graph
    * ??
