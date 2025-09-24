// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Reefscape - 2025
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.common.AlignToBranchSide;
import frc.robot.common.ElevatorPositions;

public class ManualInputInterfaces {

    // sets joystick variables to joysticks
    private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController);
    private XboxController driverControllerForRumbleOnly = new XboxController(Constants.portDriverController);
    private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController);
    private XboxController coDriverControllerForRumbleOnly = new XboxController(Constants.portCoDriverController);

    private DigitalInput climberLimSwitch = new DigitalInput(Constants.climberLimSwtichChannel);

    // subsystems needed for inputs
    private SubsystemCollection subsystemCollection = null;

    /**
     * The constructor to build this 'manual input' conduit
     */
    public ManualInputInterfaces(SubsystemCollection currentCollection) {
        subsystemCollection = currentCollection;
    }

    /**
     * A method to return the co driver controller for rumble needs
     * 
     * @return
     */
    public final XboxController getCoDriverController() {
        return coDriverControllerForRumbleOnly;
    }

    /**
     * A method to get the arcade drive X componet being input from humans
     * 
     * @return - a double value associated with the magnitude of the x componet
     */
    public double getInputArcadeDriveX() {
        return -driverController.getLeftX();
    }

    /**
     * A method to get the arcade drive Y componet being input from humans
     * 
     * @return - a double value associated with the magnitude of the y componet
     */
    public double getInputArcadeDriveY() {
        return -driverController.getLeftY();
    }

    /**
     * A method to get the spin drive X componet being input from humans
     * 
     * @return - a double value associated with the magnitude of the x componet
     */
    public double getInputSpinDriveX() {
        return driverController.getRightX();
    }

    /**
     * A method to return the Y value of the left joystick on the co-driver's
     * controller
     * 
     * @return - a double value associated with the magnitude of the y componet
     */
    public double getCoDriverLeftY() {
        return -coDriverController.getLeftY();
    }

    /**
     * A method to return the Y value of the right joystick on the co-driver's
     * controller
     * 
     * @return - a double value associated with the magnitude of the y componet
     */
    public double getCoDriverRightY() {
        return -coDriverController.getRightY();
    }

    /**
     * A method to get if the climber lim switch is pressed
     * 
     * @return - a boolean of wether the lim switch is pressed
     */
    public boolean isCLimberLimSwitchPressed() {
        return climberLimSwitch.get();
    }

    /**
     * A method to initialize various commands to the numerous buttons.
     * Need delayed bindings as some subsystems during testing won't always be
     * there.
     */
    public void initializeButtonCommandBindings() {
        // Configure the driver xbox controller bindings
        if (InstalledHardware.driverXboxControllerInstalled) {
            this.bindCommandsToDriverXboxButtons();
        }

        // Configure the co-driver xbox controller bindings
        if (InstalledHardware.coDriverXboxControllerInstalled) {
            this.bindCommandsToCoDriverXboxButtons();
        }
    }

    /**
     * Will attach commands to the Driver XBox buttons
     */
    private void bindCommandsToDriverXboxButtons() {
        if (InstalledHardware.driverXboxControllerInstalled) {
            if (this.subsystemCollection.isDriveTrainSubsystemAvailable()) {
                // Back button zeros the gyroscope (as in zero yaw)
                this.driverController.back().onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(
                                        subsystemCollection
                                                .getDriveTrainSubsystem()::zeroGyroscope),
                                new ButtonPressCommand(
                                        "driverController.back()",
                                        "zero gyroscope")));
                DataLogManager.log("FINISHED registering back button to zero gyroscope ... ");

                // Align to branch for scoring
                this.driverController.a().onTrue(
                        new ParallelCommandGroup(
                                new AlignToBranchCommand(subsystemCollection.getDriveTrainSubsystem(),
                                        subsystemCollection.getBranchDetectorSubsystem(),
                                        () -> subsystemCollection.getAlignWithBranchDirection()
                                                .getAlignWithBranchSide()),
                                new ButtonPressCommand(
                                        "driverController.a()",
                                        "Align to branch")));
                // Align to reef for scoring
                this.driverController.b().onTrue(
                        new ParallelCommandGroup(
                                new AlignWithReefCommand(subsystemCollection, false),
                                new ButtonPressCommand(
                                        "driverController.b()",
                                        "Align to reef")));
            }

            if (this.subsystemCollection.isDriveTrainPowerSubsystemAvailable()) {
                // Enable pit limiter
                this.driverController.leftTrigger().onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(
                                        subsystemCollection
                                                .getDriveTrainPowerSubsystem()::setReducedPowerReductionFactor,
                                        subsystemCollection
                                                .getDriveTrainPowerSubsystem()),
                                new ButtonPressCommand(
                                        "driverController.leftTrigger()",
                                        "ramp down to reduced speed")));

                // Disable pit limiter
                this.driverController.leftTrigger().onFalse(
                        new ParallelCommandGroup(
                                new InstantCommand(
                                        subsystemCollection
                                                .getDriveTrainPowerSubsystem()::resetPowerReductionFactor,
                                        subsystemCollection
                                                .getDriveTrainPowerSubsystem()),
                                new ButtonPressCommand(
                                        "driverController.leftTrigger()",
                                        "ramp up to default speed")));
            }

            // x button press will stop all
            this.driverController.x().onTrue(
                    new ParallelCommandGroup(
                            new AllStopCommand(
                                    this.subsystemCollection),
                            new ButtonPressCommand(
                                    "driverController.x()",
                                    "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")));

            if (this.subsystemCollection.isEndEffectorSubsystemAvailable()) {
                this.driverController.leftBumper().whileTrue(
                        new ParallelCommandGroup(
                                new ClearAlgaeCommand(this.subsystemCollection.getEndEffectorSubsystem()),
                                new ButtonPressCommand(
                                        "driverController.leftBumper()",
                                        "Remove Algae")));

                // Score Coral with EndEffector
                this.driverController.y().whileTrue(
                        new ParallelCommandGroup(
                                new ScoreCoralCommand(this.subsystemCollection.getEndEffectorSubsystem()),
                                new ButtonPressCommand(
                                        "driverController.y()",
                                        "Score Coral")));

                // Score Coral with EndEffector
                this.driverController.rightBumper().onTrue(
                        new ParallelCommandGroup(
                                new IntakeCoralCommand(this.subsystemCollection.getEndEffectorSubsystem()),
                                new ButtonPressCommand(
                                        "driverController.rightBumper()",
                                        "Intake Coral")));
            }
        }
    }

    /**
     * Will attach commands to the Co Driver XBox buttons
     */
    private void bindCommandsToCoDriverXboxButtons() {
        if (InstalledHardware.coDriverXboxControllerInstalled) {
            // x button press will stop all
            this.coDriverController.x().onTrue(
                    new ParallelCommandGroup(
                            new AllStopCommand(
                                    this.subsystemCollection),
                            new ButtonPressCommand(
                                    "coDriverController.x()",
                                    "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")));

            if (this.subsystemCollection.isElevatorSubsystemAvailable()) {
                // Move elevator to selected level (L1-L4)
                this.coDriverController.y().onTrue(
                        new ParallelCommandGroup(
                                new MoveToPositionCommand(
                                        this.subsystemCollection.getElevatorSubsystem(),
                                        this.subsystemCollection.getElevatorHeightState()::getElevatorPosition),
                                new ButtonPressCommand(
                                        "coDriverController.y()",
                                        "Move elevator to previously selected position")));

                // Move elevator to lowest height
                this.coDriverController.a().onTrue(
                        new ParallelCommandGroup(
                                new MoveToPositionCommand(this.subsystemCollection.getElevatorSubsystem(),
                                        () -> ElevatorPositions.STOW),
                                new ButtonPressCommand(
                                        "coDriverController.a()",
                                        "Move elevator to stow position")));

                // Change target elevator position to L1 (Does not actually move the elevator)
                this.coDriverController.povLeft().onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> this.subsystemCollection.getElevatorHeightState()
                                        .setElevatorPosition(ElevatorPositions.L1)),
                                new ButtonPressCommand(
                                        "coDriverController.povLeft()",
                                        "Set target elevator position to L1")));
                // Change target elevator position to L2 (Does not actually move the elevator)
                this.coDriverController.povDown().onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> this.subsystemCollection.getElevatorHeightState()
                                        .setElevatorPosition(ElevatorPositions.L2)),
                                new ButtonPressCommand(
                                        "coDriverController.povDown()",
                                        "Set target elevator position to L2")));
                // Change target elevator position to L3 (Does not actually move the elevator)
                this.coDriverController.povRight().onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> this.subsystemCollection.getElevatorHeightState()
                                        .setElevatorPosition(ElevatorPositions.L3)),
                                new ButtonPressCommand(
                                        "coDriverController.povRight()",
                                        "Set target elevator position to L3")));
                // Change target elevator position to L4 (Does not actually move the elevator)
                this.coDriverController.povUp().onTrue(
                        new ParallelCommandGroup(
                                new InstantCommand(() -> this.subsystemCollection.getElevatorHeightState()
                                        .setElevatorPosition(ElevatorPositions.L4)),
                                new ButtonPressCommand(
                                        "coDriverController.povUp()",
                                        "Set target elevator position to L4")));
                this.coDriverController.back().onTrue(
                        new ParallelCommandGroup(
                                new ZeroElevatorCommand(this.subsystemCollection.getElevatorSubsystem()),
                                new ButtonPressCommand(
                                        "coDriverController.back()",
                                        "zero elevator")));
            }

            if (this.subsystemCollection.isFunnelSubsystemAvailable()) {
                // Collapse the funnel when both Left Bumper and B are pressed
                Trigger doubleButtonTrigger = new Trigger(
                        () -> this.coDriverController.leftBumper().getAsBoolean()
                                && this.coDriverController.b().getAsBoolean());
                doubleButtonTrigger.whileTrue(
                        new ParallelCommandGroup(
                                new OpenFunnelCommand(this.subsystemCollection.getFunnelSubsystem()),
                                new ButtonPressCommand(
                                        "coDriverController leftBumper && b",
                                        "Collapse Funnel")));
                this.coDriverController.rightBumper().whileTrue(
                        new ParallelCommandGroup(
                                new CloseFunnelCommand(this.subsystemCollection.getFunnelSubsystem()),
                                new ButtonPressCommand("coDriverController right bumper",
                                        "Close Funnel")));
            }

            // Change alignment mode to left (changes the align with branch settings)
            this.coDriverController.leftTrigger().onTrue(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> subsystemCollection.getAlignWithBranchDirection()
                                    .setAlignWithBranchSide(AlignToBranchSide.LEFT)),
                            new ButtonPressCommand(
                                    "coDriverController.leftTrigger()",
                                    "Align Left")));

            // Change alignment mode to right (changes the align with branch settings)
            this.coDriverController.rightTrigger().onTrue(
                    new ParallelCommandGroup(
                            new InstantCommand(() -> subsystemCollection.getAlignWithBranchDirection()
                                    .setAlignWithBranchSide(AlignToBranchSide.RIGHT)),
                            new ButtonPressCommand(
                                    "coDriverController.rightBumper()",
                                    "Align Right")));
        }
    }
}
