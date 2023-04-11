package frc.robot.commands;
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

public class TurnAndMove extends CommandBase {

    private Swerve s_Swerve;
    private Pose2d target;
    private PIDController xPid;
    private PIDController yPid;
    private double theta;
    private PIDController thetaPid;
    private double overdrive;
    private double time;
    private Timer timer;
    private boolean hasHitAngle;

    public TurnAndMove(Swerve s, Pose2d tgt, double angle, double timeout) 
    {
        s_Swerve = s;
        addRequirements(s_Swerve);
        target = tgt.div(39.37); //coverting inches to meters
        theta = angle;
        overdrive = 0.0;
        time = timeout;
        timer = new Timer();
        hasHitAngle = false;
    }

    public void initialize()
    {
        // DRIVE

        // make moving PID
        xPid = new PIDController(Constants.AutoConstants.DriveToPositionXP, Constants.AutoConstants.DriveToPositionXI, Constants.AutoConstants.DriveToPositionXD);
        yPid = new PIDController(Constants.AutoConstants.DriveToPositionYP, Constants.AutoConstants.DriveToPositionYI, Constants.AutoConstants.DriveToPositionYD);
        //set setpoint targetxy
        xPid.setSetpoint(target.getX());
        yPid.setSetpoint(target.getY());
        //s_Swerve.resetOdometry(new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0)));

        
        // TURNING

        double curAngle = get180();
        
        if (curAngle > 170)
        {
            overdrive = -10;
        } else if (curAngle < -170) {
            overdrive = 10;
        }

        theta += overdrive;
        
        //make turning PID
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
        // DRIVE
        Pose2d cur = s_Swerve.getPose();
        Translation2d change = new Translation2d(xPid.calculate(cur.getX()), yPid.calculate(cur.getY()));
        Translation2d drivePose = normalizeSpeed(change);

        SmartDashboard.putString("curPose", Units.metersToInches(cur.getX()) + "  " + Units.metersToInches(cur.getY()));
        SmartDashboard.putString("targetPose", Units.metersToInches(target.getX()) + "  " + Units.metersToInches(target.getY()));

        // TURN
        double curAngle = get180();
        curAngle += overdrive;

        SmartDashboard.putNumber("curAngleTurnTo", curAngle);
        double changeAngle = thetaPid.calculate(curAngle);
        double driveAngle = normalizeAngle(changeAngle);
        
        driveAngle *= -1;
        if (overdrive > 0 && Math.abs(curAngle) > 100 && driveAngle > 0)
            driveAngle *= -1;
        if (overdrive < 0 && Math.abs(curAngle) > 100 && driveAngle < 0)
            driveAngle *= -1;

        if (theta > 178 && curAngle < -178) {
            theta = -theta;
            thetaPid.setSetpoint(theta);
            changeAngle = thetaPid.calculate(curAngle);
            driveAngle = normalizeAngle(changeAngle);
        } 
        else if (theta < -178 && curAngle > 178) 
        {
            theta = -theta;
            thetaPid.setSetpoint(theta);
            changeAngle = thetaPid.calculate(curAngle);
            driveAngle = normalizeAngle(changeAngle);
        }

        s_Swerve.drive(drivePose.times(Constants.AutoConstants.kPathMaxVelocity),
            driveAngle, 
            true, 
            true);
    }

    public double normalizeAngle(double angle) {
        
        if (angle > 0.19)          
            return angle = 0.19;
        else if (angle < -0.19)          
            return angle = -0.19;
        return angle;
    }

    public Translation2d normalizeSpeed(Translation2d translation) {
        
        if (Math.abs(translation.getX()) > 1 || Math.abs(translation.getY()) > 1) 
        {
            double hi = Math.max(translation.getX(), translation.getY());
            
            return new Translation2d(translation.getX() / hi, translation.getY() / hi);
        }
        return new Translation2d(translation.getX(), translation.getY());
    }

    public double calculateDistance(double x1, double y1, double x2, double y2) {
        return Math.sqrt(Math.pow((x2 - x1), 2) + Math.pow((y2 - y1), 2));
    }

    public boolean isFinished() 
    {
        Pose2d initial = s_Swerve.getPose();
        if (Math.abs(s_Swerve.getYaw().getDegrees() - theta) < Constants.AutoConstants.angleThreshold)
            hasHitAngle = true;

        if (Math.abs(calculateDistance(initial.getX(), initial.getY(), target.getX(), target.getY())) < Constants.AutoConstants.DriveToPositionThreshold &&
            hasHitAngle) //0.0254 just being some random threshold. = to 1 inch
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
