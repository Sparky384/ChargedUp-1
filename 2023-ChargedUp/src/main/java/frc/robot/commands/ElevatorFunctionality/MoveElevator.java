package frc.robot.commands.ElevatorFunctionality;

import edu.wpi.first.wpilibj.Timer;
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
        timer.stop();
        timer.reset();
    }

    public void initialize() {
        timer.stop();
        timer.reset();
    }

    public void execute() {
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
        if (timer.hasElapsed(0.08)) 
        {
            timer.stop();
            timer.reset();
            return true;
        }
        return false;
        
    }

    @Override
    public void end(boolean interruped){
        elevatorSubsystem.stop();
    }
}
