package frc.robot.commands.WristFunctionality;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class StopIntake extends CommandBase
{
        
    private Hand handSubsystem; 

    public StopIntake(Hand hand) {
        handSubsystem = hand;
        addRequirements(handSubsystem);
    }

    public void execute() {
        handSubsystem.stop(); 
    }

    public boolean isFinished(){
      return true;
    }
}
