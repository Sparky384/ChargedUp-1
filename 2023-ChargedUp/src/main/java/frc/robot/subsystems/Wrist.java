import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CANPorts;

public class Wrist extends SubsystemBase{
    private CANSparkMax motor; 

    public Wrist(){
        motor = new Spark (CANPorts.wrist); 
    }

    public void stop(){
        motor.stopMotor();
    }

    public void driveToAngle(double angle){
        motor.
    }
}
