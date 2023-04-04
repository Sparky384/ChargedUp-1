package frc.robot.commands.DriveFunctionality;


import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnTo extends CommandBase {

    private Swerve s_Swerve;
    private double theta;
    private PIDController thetaPid;
    private double overdrive;
    private double time;
    private Timer timer;

    public TurnTo(Swerve s, double angle, double timeout) 
    {
        s_Swerve = s;
        addRequirements(s_Swerve);
        theta = angle;
        overdrive = 0.0;
        time = timeout;
        timer = new Timer();
    }

    public void initialize()
    {
        double curAngle = get180();
        
        System.out.println(curAngle);

        if (curAngle > 170)
        {
            overdrive = -10;
        } else if (curAngle < -170) {
            overdrive = 10;
        }

        theta += overdrive;

        System.out.println(overdrive);
        System.out.println(theta);

        //make PID
        thetaPid = new PIDController(Constants.AutoConstants.TurnToP, 0.0, 0.0);
        //Set Target Point
        thetaPid.setSetpoint(theta);
        timer.stop();
        timer.reset();
        timer.start();
    }

    public double get180()
    {
        double curAngle = s_Swerve.getYaw().getDegrees();
        // reduce the angle  
        curAngle = curAngle % 360; 
        // force it to be the positive remainder, so that 0 <= angle < 360  
        curAngle = (curAngle + 360) % 360;  
        // force into the minimum absolute value residue class, so that -180 < angle <= 180  
        if (curAngle > 180)  
            curAngle -= 360;  

        return curAngle;
    }

    public void execute() 
    {
        double curAngle = get180();
        curAngle += overdrive;
        System.out.println("*" + curAngle);

        SmartDashboard.putNumber("curAngleTurnTo", curAngle);
        double change = thetaPid.calculate(curAngle);
        double driveAngle = normalizeSpeed(change);
        
        driveAngle *= -1;
        if (overdrive > 0 && Math.abs(curAngle) > 100 && driveAngle > 0)
            driveAngle *= -1;
        if (overdrive < 0 && Math.abs(curAngle) > 100 && driveAngle < 0)
            driveAngle *= -1;

        s_Swerve.drive(new Translation2d(0, 0),
            driveAngle, 
            true, 
            true);
    }

    public double normalizeSpeed(double angle) {
        
        if (angle > 0.15 )          
            return angle = 0.15;
        else if (angle < -0.15)          
            return angle = -0.15;
        return angle;
    }

    public boolean isFinished() 
    {
        if (Math.abs(s_Swerve.getYaw().getDegrees() - theta) < Constants.AutoConstants.angleThreshold)
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
