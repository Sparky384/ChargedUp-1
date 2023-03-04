package frc.robot.commands.WristFunctionality;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hand;
import edu.wpi.first.wpilibj.Timer;


public class IntakeAuto extends CommandBase{
    
    Timer timer = new Timer();

    private Hand handSubsystem; 

    public IntakeAuto(Hand hand) {
        handSubsystem = hand;
        addRequirements(handSubsystem);
    }

    public void initialize() {
        timer.reset();
        timer.start();
    }

    public void execute() {
        handSubsystem.rollIn(); 
    }

    public boolean isFinished() {
      if (timer.hasElapsed(Constants.AutoConstants.kAutoIntakeTimer))
        return true;
      else 
        return false;
    }

    public void end(boolean interrupted) {
        handSubsystem.stop();   
        timer.stop();
    }
}