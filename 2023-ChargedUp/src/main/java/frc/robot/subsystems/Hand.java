package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// No idea what imports go here but there needs to be some


// Subsystem initializes hardware and methods that are then going to be used in commands
public class Hand extends SubsystemBase
{
    private CANSparkMax motor; 

    public Hand() 
    {
        motor = new CANSparkMax(Constants.CANPorts.hand, MotorType.kBrushless);
    }

    public void stop()
    {
        motor.stopMotor();
    }

    public void shoot()
    {
        motor.set(0.5);
    }
    public void rollIn()
    {
        motor.set(-0.5);
    }
}
