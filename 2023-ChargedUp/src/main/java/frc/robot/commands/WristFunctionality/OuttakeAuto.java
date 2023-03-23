package frc.robot.commands.WristFunctionality;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hand;

public class OuttakeAuto extends CommandBase{
    
    Timer timer = new Timer();

    private Hand handSubsystem; 

    public OuttakeAuto(Hand hand) {
        handSubsystem = hand;
        addRequirements(handSubsystem);
    }

    public void initialize() {
        timer.stop();
        timer.reset();
        timer.start();
    }

    public void execute() {
        handSubsystem.shoot(); 
    }

    public boolean isFinished() {
      if (timer.hasElapsed(Constants.AutoConstants.kAutoShootTimer))
        return true;
      else
        return false;
    }

    public void end(boolean interrupted) {
        handSubsystem.stop();   
        timer.stop();
    }
}
