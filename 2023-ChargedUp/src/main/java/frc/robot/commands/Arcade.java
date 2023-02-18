package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveTrain;

public class Arcade extends CommandBase {
    private DriveTrain driveTrain;
    private DoubleSupplier speed;
    private DoubleSupplier turn;

    public Arcade(DriveTrain subsystem, DoubleSupplier s, DoubleSupplier t) 
    {
        driveTrain = subsystem;
        speed = s;
        turn = t;

        addRequirements(driveTrain);
    }

    public void execute() 
    {
        driveTrain.drive(speed.getAsDouble(), turn.getAsDouble());
    }

    public boolean isFinished() 
    {
        return false;
    }

    
}
