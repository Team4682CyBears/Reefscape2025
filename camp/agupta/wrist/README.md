# Wrist code

Intent:
- Used to remove algae and go to 1 specific wrist position
    - For testing, set to 0 degrees and 90 degrees

Code Origins:
- Copied code from shooter angle ted
- Copied code from climber reefscape

# Commands

- Coral Position
- Algae Remove

# Software Libraries

- For control, we use [motion magic](https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/motion-magic.html)
- The following parameters must be set when controlling using Motion Magic®
    - Cruise Velocity - peak/cruising velocity of the motion
    - Acceleration - controls acceleration and deceleration rates during the beginning and end of motion
    - Jerk (optional) - controls jerk, which is the derivative of acceleration
 