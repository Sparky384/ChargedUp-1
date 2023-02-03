import edu.wpi.first.wpilibj2.command.CommandBase;


public class PullUpWrist extends CommandBase {

    Wrist wristSubsystem; 

    public PullUpWrist (Wrist wrist) {

        wristSubsystem = wrist; 

    }

    public void execute() {
        wristSubsystem.pullUpWrist(); 

    }

    public boolean isFinished(){
        if (sensor.getValue == 0){
            return true; 
        }
    }

}


