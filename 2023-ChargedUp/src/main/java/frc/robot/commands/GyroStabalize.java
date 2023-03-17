package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class GyroStabalize extends CommandBase {
    private Swerve s_swerve;
    
    public GyroStabalize(Swerve swerve) {
        s_swerve = swerve;
        addRequirements(s_swerve);
    }

    public void execute() {
        // - = going up
        SmartDashboard.putBoolean("gyro on", true);
        double output = -(s_swerve.getRoll() * Constants.RampConstants.P);
        if (output > 1.0)
            output = 1.0;
        if (output < -1.0)
            output = -1.0;
        output *= Constants.RampConstants.maxRampSpeed;
        if (output > Constants.RampConstants.maxRampSpeed)
            output = Constants.RampConstants.maxRampSpeed;
        else if (output < -Constants.RampConstants.maxRampSpeed)
            output = -Constants.RampConstants.maxRampSpeed;

        if (Math.abs(s_swerve.getRoll()) < 3.5)
            output = 0;
        
        SmartDashboard.putNumber("gyro output", output);
        s_swerve.drive(new Translation2d(output, 0).times(Constants.Swerve.maxSpeed), 
        0, 
        false, 
        true);
    }

    public boolean isFinished() 
    {
        return false;
    }

    public void end(boolean interrupted)
    {
        SmartDashboard.putBoolean("gyro on", false);
        s_swerve.drive(new Translation2d(0, 0).times(Constants.Swerve.maxSpeed),
        0.5, 
        false, 
        true);
    }
}
