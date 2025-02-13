# Purpose of Code

This code is to test motors to move the elevator.

- Motor: Kraken Motors 
    - 2 motors
    - main & follower motor
        - set the code of main motor and then set the follower motor 
        - slow speed for testing
            - figure out speed for production later
        - one motor rotates clockwise the other is counterclockwise
        - right next to each on center of elevator
        - connecting with gears
            - belt on elevator
- SW: TalonFX
- Control: Motion Magic
    - stops it from over/undershooting
    - tolerance - 0.25 inches 
- Sensor - magsensor
    - when elevator reaches the bottom, the sensor will reset the values to stow
    - reads when elevator is released but doesn't do anything
    - may need a magnetic piece/metal to detect interference


4 presets: by pressing a button it goes to a set position
- x - stow - not used, put away
- a - l2 - 32in
- b - l3 - 48in
- y - l4 - 72in

For test:
- up/down: dpad to move elevator manually




