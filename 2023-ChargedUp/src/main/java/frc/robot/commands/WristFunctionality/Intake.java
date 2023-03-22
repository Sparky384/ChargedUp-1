package frc.robot.commands.WristFunctionality;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;


public class Intake extends CommandBase{
    
    private Hand handSubsystem; 
    //intakes cones. outtakes cubes.
    public Intake(Hand hand) {
        handSubsystem = hand;
        addRequirements(handSubsystem);
    }

    public void execute() {
        handSubsystem.rollIn(); 
    }

    public boolean isFinished(){
      return false;
    }

    @Override
    public void end(boolean interrupted) 
    {
        handSubsystem.keepIn();   
    }
}