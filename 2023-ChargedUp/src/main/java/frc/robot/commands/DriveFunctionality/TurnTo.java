package frc.robot.commands.DriveFunctionality;


import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnTo extends CommandBase {

    private Swerve s_Swerve;
    private double theta;
    private PIDController thetaPid;

    public TurnTo(Swerve s, double angle) 
    {
        s_Swerve = s;
        addRequirements(s_Swerve);
        theta = angle;
    }

    public void initialize()
    {
        //make PID
        thetaPid = new PIDController(0.0, 0.0, 0.0);
        //Set Target Point
        thetaPid.setSetpoint(theta);
        
    }

    public void execute() 
    {
        double curAngle = s_Swerve.getYaw().getDegrees();
        double change = thetaPid.calculate(curAngle);
        double driveAngle = normalizeSpeed(change);

        var drive = 0.1;
        if (theta > s_Swerve.getYaw().getDegrees())
            drive = -0.1;
        
        s_Swerve.drive(new Translation2d(0, 0),
            drive, 
            false, 
            true);
    }

    public double normalizeSpeed(double angle) {
        
        if (Math.abs(angle) > 1) 
        {            
            return angle = 1;
        }
        return angle;
    }

    public boolean isFinished() 
    {
        if (Math.abs(s_Swerve.getYaw().getDegrees() - theta) < Constants.AutoConstants.angleThreshold)
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
