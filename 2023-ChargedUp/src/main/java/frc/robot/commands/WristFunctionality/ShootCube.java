package frc.robot.commands.WristFunctionality;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Hand;

public class ShootCube extends CommandBase{

    private Hand handSubsystem; 
    public ShootCube(Hand hand) {
        handSubsystem = hand;
        addRequirements(handSubsystem);
    }

    @Override
    public void execute() {
        handSubsystem.outtakeCube(); 
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        handSubsystem.stop();   
    }
}
