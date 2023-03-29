package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

// Subsystem initializes hardware and methods that are then going to be used in commands
public class Hand extends SubsystemBase
{
    private CANSparkMax motor;

    public Hand() 
    {
        motor = new CANSparkMax(Constants.CANPorts.hand, MotorType.kBrushless);
        motor.setIdleMode(IdleMode.kCoast);
    }

    public void stop()
    {
        motor.stopMotor();
    }

    //intake for cubes
    public void shoot()
    {
        motor.set(0.5); 
    }
    
    public void keepOut() {
        motor.set(0.0);
    }

    //intake for cones.
    public void rollIn()
    {
        motor.set(-0.5); 
    }

    public void outtakeCube()
    {
        motor.set(-1); 
    }


    public void keepIn() {
        motor.set(-0.1);
    }
}
