package frc.robot.commands.DriveFunctionality;


import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class DriveToPosition extends CommandBase {

    private Swerve s_Swerve;
    private Pose2d target;
    private PIDController xPid;
    private PIDController yPid;
    private double time;
    private Timer timer;

    public DriveToPosition(Swerve s, Pose2d tgt, double timeout) 
    {
        s_Swerve = s;
        addRequirements(s_Swerve);
        target = tgt.div(39.37); //coverting inches to meters
        time = timeout;
        timer = new Timer();
    }

    public void initialize()
    {
        // make pid
        xPid = new PIDController(Constants.AutoConstants.DriveToPositionXP, 0, Constants.AutoConstants.DriveToPositionXD);
        yPid = new PIDController(Constants.AutoConstants.DriveToPositionYP, 0, Constants.AutoConstants.DriveToPositionYD);
        //set setpoint targetxy
        xPid.setSetpoint(target.getX());
        yPid.setSetpoint(target.getY());
        //s_Swerve.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));
        timer.stop();
        timer.reset();
        timer.start();
    }

    public void execute() 
    {
        Pose2d cur = s_Swerve.getPose();
        Translation2d change = new Translation2d(xPid.calculate(cur.getX()), yPid.calculate(cur.getY()));
        Translation2d drivePose = normalizeSpeed(change);

        SmartDashboard.putString("curPose", Units.metersToInches(cur.getX()) + "  " + Units.metersToInches(cur.getY()));
        SmartDashboard.putString("targetPose", Units.metersToInches(target.getX()) + "  " + Units.metersToInches(target.getY()));

        s_Swerve.drive(drivePose.times(Constants.Swerve.maxSpeed),
        0.0, 
        true, 
        true);
    }

    public Translation2d normalizeSpeed(Translation2d translation) {
        
        if (Math.abs(translation.getX()) > 1 || Math.abs(translation.getY()) > 1) 
        {
            double hi = Math.max(translation.getX(), translation.getY());
            
            return new Translation2d(translation.getX() / Math.abs(hi), translation.getY() / Math.abs(hi));
        }
        return new Translation2d(translation.getX(), translation.getY());
    }

    public double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    }

    public boolean isFinished() 
    {
        Pose2d initial = s_Swerve.getPose();
        SmartDashboard.putNumber("Distance", calculateDistance(initial.getX(), initial.getY(), target.getX(), target.getY()));

        if (Math.abs(calculateDistance(initial.getX(), initial.getY(), target.getX(), target.getY())) < Constants.AutoConstants.DriveToPositionThreshold) //0.0254 just being some random threshold. = to 1 inch
            return true;
        else if (timer.hasElapsed(time))
            return true;
        else 
            return false;
        
    }

    public void end(boolean interrupted)
    {
        s_Swerve.drive(new Translation2d(0.0, 0.0),
            0, 
            true, 
            true);
    }
}
