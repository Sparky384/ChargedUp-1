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

    @Override
    public void initialize() {
        timer.stop();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        handSubsystem.outtakeCube(); 
    }

    @Override
    public boolean isFinished() {
      if (timer.hasElapsed(Constants.AutoConstants.kAutoShootTimer))
        return true;
      else 
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        handSubsystem.stop();   
        timer.stop();
    }
}