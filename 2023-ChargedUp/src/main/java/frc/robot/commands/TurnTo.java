package frc.robot.commands;

import org.ejml.dense.row.linsol.qr.AdjLinearSolverQr_DDRM;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class TurnTo extends CommandBase {

    private Swerve s_Swerve;
    private double theta;

    public TurnTo(Swerve s, double angle) 
    {
        s_Swerve = s;
        addRequirements(s_Swerve);
        theta = angle;
    }

    public void initialize()
    {
    }

    public void execute() 
    {
        var drive = 0.1;
        if (theta > s_Swerve.getYaw().getDegrees())
            drive = -0.1;
        s_Swerve.drive(new Translation2d(0, 0),
            drive, 
            false, 
            true);
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
