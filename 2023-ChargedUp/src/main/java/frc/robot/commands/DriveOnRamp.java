package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveOnRamp extends CommandBase{
    private Swerve s_swerve;
    private boolean backward;

    public DriveOnRamp(Swerve swerve, Boolean backward) {
        s_swerve = swerve;
        addRequirements(s_swerve);
    }

    public void execute() {
        if (backward == true) { //if we're going onto the ramp while facing backward.
            s_swerve.drive(new Translation2d(0.5, 0).times(Constants.Swerve.maxSpeed).times(-1),
            0, 
            false, 
            true);
        }
        else { //if we're going onto the ramp while facing forward/towards it.
            s_swerve.drive(new Translation2d(0.5, 0).times(Constants.Swerve.maxSpeed),
            0, 
            false, 
            true);
        }
    }

    public boolean isFinished() {
        if (Math.abs(s_swerve.getPitch()) > Constants.rampThreshold) 
            return true;
        else
            return false;
    }

}
