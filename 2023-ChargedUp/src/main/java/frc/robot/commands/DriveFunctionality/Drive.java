package frc.robot.commands.DriveFunctionality;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class Drive extends CommandBase {

    private Timer timer;
    private Swerve s_Swerve;
    private double x;
    private double y;
    private double theta;
    private double time;

    public Drive(Swerve s, double xIn, double yIn, double timeIn, double angle) 
    {
        s_Swerve = s;
        addRequirements(s_Swerve);
        x = xIn;
        y = yIn;
        time = timeIn;
        theta = angle;
        timer = new Timer();
    }

    public void initialize()
    {
        timer.stop();
        timer.reset();
        timer.start();
    }

    public void execute() 
    {
        s_Swerve.drive(new Translation2d(-x, y).times(Constants.Swerve.maxSpeed),
            theta, 
            false, 
            true);
    }

    public boolean isFinished() 
    {
        System.out.println(timer.get() + "    " + time);
        if (timer.hasElapsed(time))
            return true;
        else
            return false;
    }

    public void end(boolean interrupted)
    {
        s_Swerve.drive(new Translation2d(0.0, 0.0).times(Constants.Swerve.maxSpeed),
            0, 
            false, 
            true);
    }
}
