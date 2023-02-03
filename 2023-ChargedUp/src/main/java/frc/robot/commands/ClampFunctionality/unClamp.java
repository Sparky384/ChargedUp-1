public class unClamp extends CommandBase {
    
    Hand handSubsystem; 

    public unClamp(Hand hand){
        handSubsystem = hand; 

    }
    
    public void execute(){
        handSubsystem.unClamp(); 
    }

    public boolean isFinished(){
        return true; 
    }
    
}
