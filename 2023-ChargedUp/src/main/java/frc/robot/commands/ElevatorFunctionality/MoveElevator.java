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
        timer.stop();
        timer.reset();
        System.out.println("MoveElevator");
    }

    public void initialize() {
        timer.stop();
        timer.reset();
        SmartDashboard.putBoolean("timerRunning", false);
        System.out.println("initialize");
    }

    public void execute() {
        SmartDashboard.putNumber("elevatorHeight", elevatorSubsystem.getHeight());
        SmartDashboard.putBoolean("running", true);
        elevatorSubsystem.move(elevatorHeight); 
        System.out.println("execute");
    }

    public boolean isFinished(){
        System.out.println("isFinished");
        if (elevatorSubsystem.getHeight() > 20)
            return true;

        if (Math.abs(elevatorSubsystem.getHeight() - elevatorHeight) < Constants.elevatorThreshold){
            timer.start(); 
            SmartDashboard.putBoolean("timerRunning", true);
        } else {
            SmartDashboard.putBoolean("timerRunning", false);
            timer.stop();
            timer.reset();
        }
        SmartDashboard.putNumber("diff", Math.abs(elevatorSubsystem.getHeight() - elevatorHeight));
        SmartDashboard.putNumber("cur time", timer.get());
        if (timer.hasElapsed(0.08)) 
        {
            SmartDashboard.putBoolean("timerRunning", false);
            timer.stop();
            timer.reset();
            return true;
        }
        return false;
        
    }

    @Override
    public void end(boolean interruped){
        System.out.println("end");
        elevatorSubsystem.stop();
        SmartDashboard.putBoolean("running", false);
    }
}
