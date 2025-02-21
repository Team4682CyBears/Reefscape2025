// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: RobotContainer.java
// Intent: main robot body
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.common.TestTrajectories;
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.control.AutonomousChooser;
import frc.robot.control.Constants;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {

  private SubsystemCollection subsystems = new SubsystemCollection();
  private AutonomousChooser autonomousChooser;

  public RobotContainer() {

    // init the data logging
    this.initializeDataLogging();

    // init the pdp watcher
    this.initializePowerDistributionPanelWatcherSubsystem();

    // init the camera (before drivetrain)
    this.initializeCameraSubsystem();

    // init the leds
    this.initializeLEDSubsystem();

    // init the various subsystems
    this.initializeDrivetrainSubsystem();
    this.initailizeBranchDetectorSubsystem();

    // init the climber
    this.initializeClimberSubsystem();

    // init the input system 
    this.initializeManualInputInterfaces();

    // do late binding of default commands
    this.lateBindDefaultCommands();


    AutonomousChooser.configureAutoBuilder(subsystems);
    autonomousChooser  = new AutonomousChooser(subsystems);


    // Configure the button bindings
    if(this.subsystems.isManualInputInterfacesAvailable()) {
      DataLogManager.log(">>>> Initializing button bindings.");
      this.subsystems.getManualInputInterfaces().initializeButtonCommandBindings();
      DataLogManager.log(">>>> Finished initializing button bindings.");
    }
    
    // TODO For debugging. Can remove for final competition build. 
    this.initializeDebugDashboard();

    if (subsystems.isDriveTrainSubsystemAvailable()) {
      // Path Planner Path Commands
      // commands to drive path planner test trajectories
      TestTrajectories testtrajectories = new TestTrajectories();
      /**
      SmartDashboard.putData("One Meter",
        FollowTrajectoryCommandBuilder.build(testtrajectories.oneMeter, this.subsystems.getDriveTrainSubsystem()));
      SmartDashboard.putData("Two Meter",
        FollowTrajectoryCommandBuilder.build(testtrajectories.twoMeter, this.subsystems.getDriveTrainSubsystem()));
      SmartDashboard.putData("Three Meter",
        FollowTrajectoryCommandBuilder.build(testtrajectories.threeMeter, this.subsystems.getDriveTrainSubsystem()));
      */
    }

    // Register Named Commands

    // Put command scheduler on dashboard
    SmartDashboard.putData(CommandScheduler.getInstance());

    if(this.subsystems.isDriveTrainPowerSubsystemAvailable()) {
      SmartDashboard.putData(
        "DriveForwardRobotCentric",
        new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(),
        new ChassisSpeeds(0.6, 0.0, 0.0),
        3.0));
    }
  }

  public Command getAutonomousCommand() {
    return autonomousChooser.getCommand();
  }

   /**
   * A method to init all the data logging
   */
  private void initializeDataLogging() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * A method to init the PDP watcher
   */
  private void initializePowerDistributionPanelWatcherSubsystem() {
    if(InstalledHardware.powerDistributionPanelInstalled) {
      subsystems.setPowerDistributionPanelWatcherSubsystem(new PowerDistributionPanelWatcherSubsystem());
      DataLogManager.log("SUCCESS: initializePowerDistributionPanelWatcherSubsystem");
    }
    else {
      DataLogManager.log("FAIL: initializePowerDistributionPanelWatcherSubsystem");
    }
  }
  /**
   * A method to init items for the debug dashboard
   */
  private void initializeDebugDashboard() {
    SmartDashboard.putData("Debug: CommandScheduler", CommandScheduler.getInstance());
  }

  /**
   * A method to init the drive train
   */
  private void initializeDrivetrainSubsystem() {
    if(InstalledHardware.drivetrainInstalled){
      // The robot's subsystems and commands are defined here...
      subsystems.setDriveTrainSubsystem(new DrivetrainSubsystem(subsystems));
      subsystems.setDriveTrainPowerSubsystem(new DrivetrainPowerSubsystem(subsystems.getDriveTrainSubsystem()));
      DataLogManager.log("SUCCESS: initializeDrivetrain");

      // Set up the default command for the drivetrain.
      // The controls are for field-oriented driving:
      // Left stick Y axis -> forward and backwards movement
      // Left stick X axis -> left and right movement
      // Right stick X axis -> rotation
      subsystems.getDriveTrainSubsystem().setDefaultCommand(
        new DefaultDriveCommand(
          subsystems.getDriveTrainSubsystem(),
          () -> RobotContainer.modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> RobotContainer.modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -RobotContainer.modifyAxisSquare(subsystems.getManualInputInterfaces().getInputSpinDriveX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));
    }
    else {
      DataLogManager.log("FAIL: initializeDrivetrain");
    }
  }

  private void initializeClimberSubsystem() {
    // TODO: Set default command to control climber with joystick
  }

  /**
   * A method to init the CameraSubsystem
   */
  private void initializeCameraSubsystem(){
    if(InstalledHardware.limelightInstalled) {
      subsystems.setCameraSubsystem(new CameraSubsystem());
      DataLogManager.log("SUCCESS: initializeCamera");
    }
    else {
      DataLogManager.log("FAIL: initializeCamera");
    }
  }

  /**
   * A method to init the BranchDetectorSubsystem
   */
  private void initailizeBranchDetectorSubsystem(){
    if(InstalledHardware.branchDetectorInstalled) {
      subsystems.setBranchDetectorSubsystem(new BranchDetectorSubsystem());
      DataLogManager.log("SUCCESS: initailize Branch Detector Subsystem");
    }
    else {
      DataLogManager.log("FAIL: initialize Branch Detector Subsystem");
    }
  }

  /**
   * A method to init the LEDSubsystem
   */
  private void initializeLEDSubsystem(){
    if(InstalledHardware.LEDSInstalled){
      subsystems.setLEDSubsystem(new LEDSubsystem(Constants.ledCanID, Constants.ledStripType));
      System.out.println("SUCCESS: initializeLEDS");
    }
    else {
      System.out.println("FAIL: initializeLEDS");
    }
  }

  /**
   * A method to init the input interfaces
   */
  private void initializeManualInputInterfaces() {
    // note: in this case it is safe to build the interfaces if only one of the controllers is present
    // because button binding assignment code checks that each is installed later (see: initializeButtonCommandBindings)
    if(InstalledHardware.driverXboxControllerInstalled ||
      InstalledHardware.coDriverXboxControllerInstalled) {
      subsystems.setManualInputInterfaces(new ManualInputInterfaces(subsystems));
      DataLogManager.log("SUCCESS: initializeManualInputInterfaces");
    }
    else {
      DataLogManager.log("FAIL: initializeManualInputInterfaces");
    }
  }

  /**
   * A method to late binding of default commands
   */
  private void lateBindDefaultCommands() {
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      }
      else {
        return (value + deadband) / (1.0 - deadband);
      }
    }
    else {
      return 0.0;
    }
  }

  private static double modifyAxisSquare(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Joystick input exponent
    value = Math.copySign(value * value, value);

    return value;
  }
}
