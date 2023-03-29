package frc.robot.commands;

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
    private String camera;

    public AutoCenter(Swerve swerve2, Limelight l, boolean goRight, String cam)
    {
        swerve = swerve2;
        lime = l;
        right = goRight;
        camera = cam;

        addRequirements(swerve);
    }

    @Override
    public void execute()
    {
        if (lime.getX(camera) < lime.errorNum) 
        {
            double translationVal = 0.6;
            if (!right)
                translationVal *= -1;
        
            swerve.drive( // drive in a direction until we see a target
                new Translation2d(0.0, translationVal).times(Constants.Swerve.maxSpeed),
                0.0, 
                false, 
                true
            );
        }
        else // sees a target
        {
            double translationVal = 0.6;
            if (lime.getX(camera) < 0.0)
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
        if (Math.abs(lime.getX(camera)) < Constants.LimelightConstants.limelightDeadband)
            return true;
        return false;
    }
}
