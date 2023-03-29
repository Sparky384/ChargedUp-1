package frc.robot.commands.DriveFunctionality;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveTo extends CommandBase {

    private Swerve s_Swerve;
    private double xPoint;
    private double yPoint;
    private double xSpeed;
    private double ySpeed;

    public DriveTo(Swerve s, double xIn, double yIn, double xSp, double ySp) 
    {
        s_Swerve = s;
        addRequirements(s_Swerve);
        xPoint = xIn / 39.37;
        yPoint = yIn / 39.37;
        xSpeed = xSp;
        ySpeed = ySp;
    }

    public void initialize()
    {
        s_Swerve.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));
    }

    public void execute() 
    {
        double driveX = xSpeed;
        if (xPoint > s_Swerve.getPose().getX())
            driveX = -xSpeed;
        if (Math.abs(xPoint - s_Swerve.getPose().getX()) < 0.0254)
            driveX = 0;
        double driveY = ySpeed;
        if (yPoint > s_Swerve.getPose().getY())
            driveY = -ySpeed;
        if (Math.abs(yPoint - s_Swerve.getPose().getY()) < 0.0254)
            driveY = 0;
        s_Swerve.drive(new Translation2d(driveX, driveY).times(Constants.Swerve.maxSpeed),
            0.0, 
            false, 
            true);
    }

    public boolean isFinished() 
    {
        if (Math.abs(xPoint - s_Swerve.getPose().getX()) < 0.0254 && Math.abs(yPoint - s_Swerve.getPose().getY()) < 0.0254)
            return true;
        else 
            return false;
        
    }

    public void end(boolean interrupted)
    {
        s_Swerve.drive(new Translation2d(0.0, 0.0),
            0, 
            false, 
            true);
    }
}
