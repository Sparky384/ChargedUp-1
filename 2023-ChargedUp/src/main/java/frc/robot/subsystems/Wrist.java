package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase{
    private CANCoder wristEncoder;
    private WPI_TalonFX wristMotor;
    private boolean wristMoving;

    public Wrist(){
        wristEncoder = new CANCoder(Constants.CANPorts.wristCancoder);
        wristMotor = new WPI_TalonFX(Constants.CANPorts.wristMotor);
        wristMoving = false;
        wristMotor.setSensorPhase(true);
        wristMotor.config_kP(0, Constants.PIDValues.wristP); //1.2
        wristMotor.config_kI(0, Constants.PIDValues.wristI);
        wristMotor.config_IntegralZone(0, Constants.PIDValues.wristIZone);
        wristMotor.config_kD(0, Constants.PIDValues.wristD); //3.2
        wristMotor.config_kF(0, 0.0);
        
        wristMotor.set(ControlMode.PercentOutput, 0.0);
        wristMotor.setInverted(true);
        wristMotor.setNeutralMode(NeutralMode.Brake);

        wristMotor.configClosedLoopPeakOutput(0, 0.28); 
        wristMotor.configForwardSoftLimitEnable(true);
        wristMotor.configForwardSoftLimitThreshold(Constants.Subsys.wristLowerLimit); 
        wristMotor.configReverseSoftLimitEnable(true);
        wristMotor.configReverseSoftLimitThreshold(Constants.Subsys.wristUpperLimit);
        
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.Subsys.timeOutMs);
        wristMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.Subsys.timeOutMs);
        
        wristMotor.configNominalOutputForward(0, Constants.Subsys.timeOutMs);
        wristMotor.configNominalOutputReverse(0, Constants.Subsys.timeOutMs);
        wristMotor.configPeakOutputForward(1, Constants.Subsys.timeOutMs);
        wristMotor.configPeakOutputReverse(-1, Constants.Subsys.timeOutMs);
        wristMotor.configMotionAcceleration(3500, 0);
        wristMotor.configMotionCruiseVelocity(3500, 0);
        
                
    }
    
    public CommandBase wristMotionMagic(double finalPositionInDegrees) {
        return runOnce(() -> {System.out.println("GO NOW======="); wristMoving = true; wristMotor.set(TalonFXControlMode.MotionMagic, convertDegreesToCts(finalPositionInDegrees), DemandType.ArbitraryFeedForward, 0.0);})
        .andThen(Commands.waitUntil(() -> Math.abs(wristEncoder.getAbsolutePosition() - finalPositionInDegrees) < Constants.Subsys.wristThreshold).withTimeout(2.5)) //2.3
        .andThen(runOnce(() -> {wristMoving = false;}));
    }

    public CommandBase wristUp() {
        return run(() -> {wristMoving = true; wristMotor.set(TalonFXControlMode.PercentOutput, 0.2);}).
        finallyDo(interrupted -> {wristMoving = false;}).
        withName("driveWrist");
    }
    

    public CommandBase wristDown() {
        return run(() -> {wristMoving = true; wristMotor.set(TalonFXControlMode.PercentOutput, -0.2);}).
        finallyDo(interrupted -> {wristMoving = false;}).
        withName("driveWrist");
    }

    public CommandBase wristStick(DoubleSupplier stick) {
        
        return run(() -> {wristMotor.set(TalonFXControlMode.PercentOutput, (stick.getAsDouble()) / 2);}). 
        finallyDo(interrupted -> wristMotor.set(TalonFXControlMode.PercentOutput, 0.0)).
        withName("driveWrist");
    }

    public double convertDegreesToCts(double degrees) {
        return (degrees * 11.4) + .603;
    }

    public double calculateFF(double currentAngle){
        double armLength = Constants.Subsys.wristLengthMain; 
        double armWeight = Constants.Subsys.wristWeightMain; 

        //Redline Motor
        double motorOhms = Constants.Subsys.wristMotorOhms;
        double motorTorque = Constants.Subsys.wristMotorTorque; //lb * in
        double motorStallCurrent = Constants.Subsys.wristMotorStallCurrent; //Amps

        double gearBox = Constants.Subsys.wristMotorGearbox;
        double kT = (motorTorque/motorStallCurrent);

        double kF = ((armLength * armWeight * motorOhms) / (kT * gearBox)) * Math.cos(Math.toRadians((currentAngle)));

        return kF;
    }

    public void periodic() {
        if (!wristMoving)
        {
            if (wristEncoder.getAbsolutePosition() > 100)
                wristMotor.set(TalonFXControlMode.PercentOutput, 0.22186 * Math.cos(Math.toRadians(wristEncoder.getAbsolutePosition())));
            else if (wristEncoder.getAbsolutePosition() > 50)
                wristMotor.set(TalonFXControlMode.PercentOutput, 0.22186 * Math.cos(Math.toRadians(wristEncoder.getAbsolutePosition())));
            else if (wristEncoder.getAbsolutePosition() < 0)
                wristMotor.set(TalonFXControlMode.PercentOutput, 0.061 * Math.cos(Math.toRadians(wristEncoder.getAbsolutePosition())));
            else
                wristMotor.set(TalonFXControlMode.PercentOutput, 0.0788 * Math.cos(Math.toRadians(wristEncoder.getAbsolutePosition())));
        }
        SmartDashboard.putBoolean("in command", wristMoving); 
        SmartDashboard.putNumber("wristPosition", wristEncoder.getAbsolutePosition());

    }
    
}
