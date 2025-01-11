package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.reefTofSensor;
import frc.robot.Constants;

public class ImplementationTOF extends SubsystemBase {
    private reefTofSensor reefCoralSensor = null;

    public ImplementationTOF(){
        reefCoralSensor = new reefTofSensor(Constants.reefTofSensorCanID);
    }

    @Override
    public void periodic() {
        if(this.reefCoralSensor != null){
            this.reefCoralSensor.publishTelemetery();
        }
    }
}
