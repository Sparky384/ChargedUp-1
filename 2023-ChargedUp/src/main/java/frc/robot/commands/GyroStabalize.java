package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class GyroStabalize extends CommandBase {
    private Swerve s_swerve;
    
    public GyroStabalize(Swerve swerve) {
        s_swerve = swerve;
        addRequirements(s_swerve);
    }

    public void execute() {
        double output = (s_swerve.getPitch() * Constants.RampConstants.P) * Constants.RampConstants.maxRampSpeed;
        if (output > Constants.RampConstants.maxRampSpeed)
            output = Constants.RampConstants.maxRampSpeed;
        else if (output < -Constants.RampConstants.maxRampSpeed)
            output = -Constants.RampConstants.maxRampSpeed;
        s_swerve.drive(new Translation2d(output, 0).times(Constants.Swerve.maxSpeed), 
        0, 
        false, 
        true);
    }

    public boolean isFinished() {
        return false;
    }
}
