# What we are doing

We are setting up the motor subsystem for the elevator


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
