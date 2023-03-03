package frc.robot.commands.WristFunctionality;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Wrist;


public class RotateWrist extends CommandBase {

    private Wrist wristSubsystem; 
    private double wristAngle;

    public RotateWrist (Wrist wrist, double initangle) {

        wristAngle = initangle;
        wristSubsystem = wrist; 
        addRequirements(wristSubsystem);
    }

    public void execute() {
        SmartDashboard.putNumber("wristDistance", wristSubsystem.getAngle());
        SmartDashboard.putBoolean("wristRunner", true);
        wristSubsystem.driveToAngle(wristAngle); 

    }

    public boolean isFinished(){
        if (Math.abs(wristSubsystem.getAngle() - wristAngle) < Constants.doubleThreshold){
            return true; 
        } else {
            return false;
        }
    }
    @Override
    public void end(boolean interruped){
        wristSubsystem.stop();
        SmartDashboard.putBoolean("wristRunner", false);
    }

}

