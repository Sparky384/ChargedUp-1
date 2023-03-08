package frc.robot.commands.ElevatorFunctionality;

import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class ManualElevator extends CommandBase{

    private Elevator s_Elevator;
    private DoubleSupplier translationSup; // Should be controller stick X

    public ManualElevator(Elevator elevator, DoubleSupplier translationSup)
    {
        s_Elevator = elevator;
        this.translationSup = translationSup;
    }

    @Override
    public void execute() {
        // Apply deadband to stick
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
        
        // Check if current height is at least greater than the lowest height constant and at least lesser than the highest height constant
        if (s_Elevator.getHeight() <= Constants.Subsys.elevatorHigh && s_Elevator.getHeight() >= Constants.Subsys.elevatorLow)
            s_Elevator.drive(translationVal * Constants.elevatorSpeedLimit); // Pass in speed * speed limiter constant to drive
        else
            s_Elevator.drive(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    // Not too fast
    // Have min and max
    // We don't know starting height

}
