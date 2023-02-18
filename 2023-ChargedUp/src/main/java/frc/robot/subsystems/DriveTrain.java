package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveTrain extends SubsystemBase{
    private WPI_TalonFX frontLeft;
    private WPI_TalonFX backLeft;
    private WPI_TalonFX frontRight;
    private WPI_TalonFX backRight;
    private DifferentialDrive diff;

    public DriveTrain() 
    {
        frontLeft = new WPI_TalonFX(Constants.CANPorts.frontLeftDrive);
        backLeft = new WPI_TalonFX(Constants.CANPorts.rearLeftDrive);
        frontRight = new WPI_TalonFX(Constants.CANPorts.frontRightDrive);
        backRight = new WPI_TalonFX(Constants.CANPorts.rearRightDrive);
        diff = new DifferentialDrive(frontLeft, frontRight);
        
        frontRight.setInverted(true);
        backRight.setInverted(true);

        backLeft.follow(frontLeft);
        backRight.follow(frontRight);
    }

    public void drive(double forward, double turn) 
    {
        diff.arcadeDrive(forward, turn);
    }

    public void stop() 
    {
        frontLeft.stopMotor();
        frontRight.stopMotor();
    }
}
