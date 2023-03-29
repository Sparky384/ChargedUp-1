package frc.robot.commands.DriveFunctionality;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveOverRamp extends CommandBase{
    private Swerve s_swerve;
    private boolean backward;
    private Timer timer;

    public DriveOverRamp(Swerve swerve, Boolean back) {
        s_swerve = swerve;
        timer = new Timer();
        addRequirements(s_swerve);
        backward = back;
    }

    public void initialize()
    {
        timer.stop();
        timer.reset();
        timer.start();
    }

    public void execute() {
        SmartDashboard.putBoolean(" on", true);
        if (backward == true) { //if we're going onto the ramp while facing backward.
            s_swerve.drive(new Translation2d(-0.40, 0).times(Constants.Swerve.maxSpeed),
            0, 
            false, 
            true);
        }
        else { //if we're going onto the ramp while facing forward/towards it.
            s_swerve.drive(new Translation2d(0.40, 0).times(Constants.Swerve.maxSpeed),
            0, 
            false, 
            true);
        }
    }

    public boolean isFinished() {        
        SmartDashboard.putBoolean(" on", false);

        if (timer.hasElapsed(3.25))
            return true;
        else 
            return false;
    }

}
