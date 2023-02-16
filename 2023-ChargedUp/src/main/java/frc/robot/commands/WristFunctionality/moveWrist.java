package frc.robot.commands.WristFunctionality;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;


public class MoveWrist extends CommandBase {

    private Wrist wristSubsystem; 
    private double wristAngle;

    public MoveWrist (Wrist wrist, double initangle) {

        wristAngle = initangle;
        wristSubsystem = wrist; 
        addRequirements(wristSubsystem);
    }

    public void execute() {
        wristSubsystem.driveToAngle(wristAngle); 

    }

    public boolean isFinished(){
        if (Math.abs(wristSubsystem.getAngle() - wristAngle) < Constants.doubleThreshold){
            return true; 
        } else {
            return false;
        }
    }

}