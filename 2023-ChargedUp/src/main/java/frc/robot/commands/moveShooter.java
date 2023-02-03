package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class moveShooter extends CommandBase{

    Hand handSubsystem; 
    public moveShooter(Hand hand){
        handSubsystem = hand; 
        
    }

    public void execute(){
        handSubsystem.moveShooter(1);
    }

    public void isFinished(){
        return true;
    }
    
}
