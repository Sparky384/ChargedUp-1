import edu.wpi.first.wpilibj2.command.CommandBase;


public class PullUpWrist extends CommandBase {

    Wrist wristSubsystem; 

    public PullUpWrist (Wrist wrist) {

        wristSubsystem = wrist; 

    }

    public void execute() {
        wristSubsystem.pullDownWrist(); 

    }

    public boolean isFinished(){
        if (sensor.getValue == 180){
            return true; 
        }
    }

}


