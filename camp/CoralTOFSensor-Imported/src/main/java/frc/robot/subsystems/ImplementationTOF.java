package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.reefTofSensor;

public class ImplementationTOF extends SubsystemBase{
    private reefTofSensor reefCoralSensor;

    public ImplementationTOF(int CANId){
        this.reefCoralSensor = new reefTofSensor(CANId);
        System.out.print("Implemenetation tof attempted to configure!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    }

    public boolean isCoralDetected(){
        return reefCoralSensor.isCoralDetected();
    }

    @Override
    public void periodic() {
        if(this.reefCoralSensor != null){
            this.reefCoralSensor.publishTelemetery();
        }
    }

}
