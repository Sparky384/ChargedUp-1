package frc.robot.commands.WristFunctionality;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Hand;

public class OuttakeAuto extends CommandBase{
    
    Timer timer = new Timer();
    private double time;

    private Hand handSubsystem; 

    public OuttakeAuto(Hand hand, double t) {
        handSubsystem = hand;
        addRequirements(handSubsystem);
        time = t;
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
      if (timer.hasElapsed(time))
        return true;
      else
        return false;
    }

    public void end(boolean interrupted) {
        handSubsystem.stop();   
        timer.stop();
    }
}
