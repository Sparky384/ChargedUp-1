package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//The method addRequirements(Subsystem...) in the type CommandBase is not applicable for the arguments (Elevator)Java(67108979)
public class Elevator extends SubsystemBase{
    private CANSparkMax motorOne; 
    private CANSparkMax motorTwo; 

    public Elevator(){
        motorOne = new CANSparkMax(Constants.CANPorts.elevatorLeft, MotorType.kBrushless);
        motorTwo = new CANSparkMax(Constants.CANPorts.elevatorRight, MotorType.kBrushless); 
        motorTwo.follow(motorOne);
        motorOne.getPIDController().setP(Constants.PIDValues.elevatorOneP); 
        motorOne.getPIDController().setI(Constants.PIDValues.elevatorOneI);
        motorOne.getPIDController().setD(Constants.PIDValues.elevatorOneD);
    }

    public void stop(){
        motorOne.stopMotor(); 
    }

    public void move(double height){
        height = height; //replace this with conversion function
        motorOne.getPIDController().setReference(height, CANSparkMax.ControlType.kPosition); 
    }

    public double getHeight() {
        double height = motorOne.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
        height = height; //replace this with conversion function
        return height;
    }
}
