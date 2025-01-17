package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.reefTofSensor;
import frc.robot.Constants;

public class ImplementationTOF extends SubsystemBase{
    private reefTofSensor reefCoralSensor = null;

    public ImplementationTOF(reefTofSensor reefCoralSensor){
        this.reefCoralSensor = reefCoralSensor;
        System.out.print("Implemenetation tof attempted to configure!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    @Override
    public void periodic() {
        //this.reefCoralSensor.blinkSensor();
        if(this.reefCoralSensor != null){
            this.reefCoralSensor.publishTelemetery();
        }
    }

}
