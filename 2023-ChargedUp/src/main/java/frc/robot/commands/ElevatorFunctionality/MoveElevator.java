package frc.robot.commands.ElevatorFunctionality;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;


public class MoveElevator extends CommandBase {

    private Elevator elevatorSubsystem; 
    private double elevatorHeight;
    private Timer timer;

    public MoveElevator(Elevator elevator, double targetHeight) {
        elevatorHeight = targetHeight;
        timer = new Timer();
        elevatorSubsystem = elevator;
        addRequirements(elevatorSubsystem);
    }

    public void execute() {
        SmartDashboard.putNumber("elevatorHeight", elevatorSubsystem.getHeight());
        SmartDashboard.putBoolean("running", true);
        elevatorSubsystem.move(elevatorHeight); 
    }

    public boolean isFinished(){

        if (elevatorSubsystem.getHeight() > 20)
            return true;

        if (Math.abs(elevatorSubsystem.getHeight() - elevatorHeight) < Constants.elevatorThreshold){
            timer.start(); 
        } else {
            timer.stop();
            timer.reset();
        }

        if (timer.hasElapsed(0.3)) {
            timer.stop();
            timer.reset();
            return true;
        } else
        return false;
    }

    @Override
    public void end(boolean interruped){
        elevatorSubsystem.stop();
        SmartDashboard.putBoolean("running", false);
    }
}
