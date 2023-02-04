package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;


public class Shoot extends CommandBase{
    
    private Hand handSubsystem; 

    public Shoot(Hand hand) {
        handSubsystem = hand;
        addRequirements(handSubsystem);
    }

    public void execute() {
        handSubsystem.shoot(); 
    }

    public boolean isFinished(){
      return false;
    }

    public void end(boolean interrupted)
    {
        handSubsystem.stop();
    }
}