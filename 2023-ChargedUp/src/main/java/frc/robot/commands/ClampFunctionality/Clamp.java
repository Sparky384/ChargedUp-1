public class Clamp extends CommandBase{
    
    Hand handSubsystem; //creates a variable handSubsystem that is an object of the Hand class in subsystems

    public Clamp(Hand hand){
        handSubsystem = hand; 
    }

    public void execute(){
        handSubsystem.clamp();
    }

    public boolean isFinished(){
        return true; // assumption: execute will be successful because of the reliability of pnematics
    }
}
