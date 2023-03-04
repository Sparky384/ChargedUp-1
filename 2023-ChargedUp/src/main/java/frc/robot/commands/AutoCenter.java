package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;

public class AutoCenter extends CommandBase
{
    private Limelight lime;
    private Swerve swerve;
    private boolean right;

    public AutoCenter(Swerve s, Limelight l, boolean goRight)
    {
        swerve = s;
        lime = l;
        right = goRight;

        addRequirements(swerve);
        // does not need to require limelight, all it does is read from it never write to it
    }

    @Override
    public void execute()
    {
        if (lime.getX() < lime.errorNum) // no target seen
        {
            double translationVal = MathUtil.applyDeadband(0.6, Constants.stickDeadband);
            if (!right)
                translationVal *= -1;
        
            swerve.drive( // drive in a direction until we see a target
                new Translation2d(translationVal, 0.0).times(Constants.Swerve.maxSpeed), 
                0.0, 
                false, 
                true
            );
        }
        else // sees a target
        {
            double translationVal = MathUtil.applyDeadband(0.6, Constants.stickDeadband);
            if (lime.getX() < 0.0)
                translationVal *= -1;
            
            swerve.drive( // drive to center on that target
                new Translation2d(translationVal, 0.0).times(Constants.Swerve.maxSpeed), 
                0.0, 
                false, 
                true
            );
        }
    }

    @Override
    public boolean isFinished()
    {
        if (Math.abs(lime.getX()) < Constants.limelightDeadband)
            return true;
        return false;
    }
}
