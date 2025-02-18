# What we are doing

We are setting up the motor subsystem for the elevator

1. Bind buttons a/b to wind/unwind the motor
2. Configure TalonFX motor 
3. Run a duty cycle to set motor speed
3. Connect the motors to controller inputs
5. Create a stop command for motor


Motor Controller: TalonFX


Subsystem
- Motor
- Config --> Get from Jasper's TOF project
    - TOF uses voltage controller, this subsystem uses duty cycle control

ControllerMode
- DutyCycleOut
- +/-!
- Wind --> +0.4
- Unwind --> -0.4
- StopMotor --> motor.StopMotor()

Command
- Unwind <-- [A] While true
- Wind <-- [B] While true

Default Command:
- STOP
