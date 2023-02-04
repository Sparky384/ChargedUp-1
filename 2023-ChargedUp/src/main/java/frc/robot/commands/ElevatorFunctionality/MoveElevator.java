package frc.robot.commands.ElevatorFunctionality;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


public class MoveElevator extends CommandBase {

    private Elevator elevatorSubsystem; 
    private double elevatorHeight;

    public MoveElevator(Elevator elevator, double initHeight) {
        elevatorHeight = initHeight;
        elevatorSubsystem = elevator;
        addRequirements(elevatorSubsystem);
    }

    public void execute() {
        elevatorSubsystem.move(elevatorHeight); 
    }

    public boolean isFinished(){
        if (Math.abs(elevatorSubsystem.getHeight() - elevatorHeight) < Constants.doubleThreshold){
            return true; 
        } else {
            return false;
        }
    }
}
