package frc.robot.commands.ElevatorFunctionality;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualElevator extends CommandBase{

    /*private Elevator s_Elevator;
    private DoubleSupplier translationSup; // Should be controller stick X

    public ManualElevator(Elevator elevator, DoubleSupplier translationSup)
    {
        s_Elevator = elevator;
        this.translationSup = translationSup;
        addRequirements(s_Elevator);
    }

    @Override
    public void execute() {
        // Apply deadband to stick
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        translationVal *= Constants.elevatorSpeedLimit;
        SmartDashboard.putNumber("elevator speed ", translationVal);

        if (s_Elevator.getHeight() >= Constants.Subsys.elevatorHigh && translationVal > 0)
            s_Elevator.drive(Constants.PIDValues.elevatorOneF);
        else if (translationVal > Constants.PIDValues.elevatorOneF)
            s_Elevator.drive(translationVal);
        else if (s_Elevator.getHeight() <= Constants.Subsys.elevatorLow && translationVal < 0)
            s_Elevator.drive(Constants.PIDValues.elevatorOneF);
        else if (translationVal < -Constants.PIDValues.elevatorOneF)
            s_Elevator.drive(Constants.PIDValues.elevatorOneF + (translationVal * Constants.PIDValues.elevatorOneF));
        else
            s_Elevator.drive(Constants.PIDValues.elevatorOneF);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    // Not too fast
    // Have min and max
    // We don't know starting height
*/
}
