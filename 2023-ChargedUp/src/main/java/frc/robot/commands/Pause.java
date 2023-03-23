package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class Pause extends CommandBase {

    private Timer timer;
    private double t;

    public Pause(double ti)
    {
        t = ti;
        timer = new Timer();
    }

    public void initialize()
    {
        SmartDashboard.putBoolean("pause", true);
        timer.stop();
        timer.reset();
        timer.start();
    }

    public boolean isFinished()
    {
        SmartDashboard.putBoolean("pause", false);
        if (timer.hasElapsed(t))
            return true;
        return false;
    }
}
