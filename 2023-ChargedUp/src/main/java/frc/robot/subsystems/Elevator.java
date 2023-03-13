package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
//The method addRequirements(Subsystem...) in the type CommandBase is not applicable for the arguments (Elevator)Java(67108979)
public class Elevator extends SubsystemBase {
    private WPI_TalonFX motorOne; 
    private WPI_TalonFX motorTwo; 
    //private Timer pidTimer;


    public Elevator(){
        motorOne = new WPI_TalonFX(Constants.CANPorts.elevatorLeft);
        motorTwo = new WPI_TalonFX(Constants.CANPorts.elevatorRight);
        //pidTimer = new Timer();

        motorTwo.follow(motorOne);
        motorOne.setSelectedSensorPosition(0.0);
        motorOne.setNeutralMode(NeutralMode.Brake);
        motorTwo.setNeutralMode(NeutralMode.Brake);
        // initialize SPARK MAX with CAN ID
        motorOne.config_kP(0, Constants.PIDValues.elevatorOneP);
        motorOne.config_kP(0, Constants.PIDValues.elevatorOneI);
        motorOne.config_kP(0, Constants.PIDValues.elevatorOneD);
        motorOne.config_kF(0, Constants.PIDValues.elevatorOneF);
        motorOne.configClosedLoopPeakOutput(0, 0.45);
        //m_encoder.setInverted(true);
    }

    public void stop(){
        motorOne.set(ControlMode.PercentOutput, 0.0); 
    }

    public void move(double height){
        //height /= Constants.ConversionValues.elevatorConversionFunction; //uses encoder counts.
        double difference = height - getHeight(); 
        double output = difference * Constants.PIDValues.elevatorOneP;
        output += Constants.PIDValues.elevatorOneF; 
        if (output > Constants.PIDValues.elevatorMaxSpeed)
            output = Constants.PIDValues.elevatorMaxSpeed;
        else if (output < Constants.PIDValues.elevatorMinSpeed)
            output = Constants.PIDValues.elevatorMinSpeed;
        motorOne.set(ControlMode.PercentOutput, output);
    }

    public void drive(double speed)
    {
        motorOne.set(ControlMode.PercentOutput, speed);
    }

    public double getHeight() {
        double height = motorOne.getSelectedSensorPosition();// was multiplying get position by negative 1
        height *= Constants.ConversionValues.elevatorConversionFunction; //uses inches (to display on screen)
        return height;
    }


    public void periodic() {
        SmartDashboard.putNumber("height", getHeight());
        //SmartDashboard.putNumber("elevatorPosition", m_encoder.getPosition());
        SmartDashboard.putNumber("elevatorSpeed", getHeight());
    }
}

