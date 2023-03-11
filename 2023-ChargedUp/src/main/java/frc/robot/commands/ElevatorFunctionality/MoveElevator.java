package frc.robot.commands.ElevatorFunctionality;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


public class MoveElevator extends CommandBase {

    private Elevator elevatorSubsystem; 
    private double elevatorHeight;

    public MoveElevator(Elevator elevator, double targetHeight) {
        elevatorHeight = targetHeight;
        elevatorSubsystem = elevator;
        addRequirements(elevatorSubsystem);
    }

    public void execute() {
        SmartDashboard.putNumber("elevatorHeight", elevatorSubsystem.getHeight());
        SmartDashboard.putBoolean("running", true);
        elevatorSubsystem.move(elevatorHeight); 
    }

    public boolean isFinished(){
        if (Math.abs(elevatorSubsystem.getHeight() - elevatorHeight) < Constants.elevatorThreshold){
            return true; 
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interruped){
        elevatorSubsystem.stop();
        SmartDashboard.putBoolean("running", false);
    }
}
