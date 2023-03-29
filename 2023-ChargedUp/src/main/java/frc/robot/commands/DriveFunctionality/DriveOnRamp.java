package frc.robot.commands.DriveFunctionality;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveOnRamp extends CommandBase{
    private Swerve s_swerve;
    private boolean backward;

    public DriveOnRamp(Swerve swerve, Boolean back) {
        s_swerve = swerve;
        addRequirements(s_swerve);
        backward = back;
    }

    public void execute() {
        SmartDashboard.putBoolean("drive on", true);
        if (backward == true) { //if we're going onto the ramp while facing backward.
            s_swerve.drive(new Translation2d(-0.42, 0).times(Constants.Swerve.maxSpeed),
            0, 
            false, 
            true);
        }
        else { //if we're going onto the ramp while facing forward/towards it.
            s_swerve.drive(new Translation2d(0.42, 0).times(Constants.Swerve.maxSpeed),
            0, 
            false, 
            true);
        }
    }

    public boolean isFinished() {
        if (Math.abs(s_swerve.getRoll()) > Constants.rampThreshold) 
            return true;
        else
            return false;
    }

    public void end(boolean interrupted)
    {
        SmartDashboard.putBoolean("drive on", false);
        if (interrupted)
        {
            s_swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
            0.5, 
            false, 
            true);
        }
    }

}
