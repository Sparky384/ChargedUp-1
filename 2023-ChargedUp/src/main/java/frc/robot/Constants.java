package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double doubleThreshold = 1.0;
    public static final double rampThreshold = 20.0; // in degrees

    public static final class Swerve {
        public static final int pigeonID = 0;
        public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

        public static final COTSFalconSwerveConstants chosenModule =
            COTSFalconSwerveConstants.SDSMK4i(COTSFalconSwerveConstants.driveGearRatios.SDSMK4i_L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(22.629); 
        public static final double wheelBase = Units.inchesToMeters(22.77);
        public static final double wheelCircumference = chosenModule.wheelCircumference;    // TODO: make sure this is right

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.angleMotorInvert;
        public static final boolean driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;
        public static final double angleKF = chosenModule.angleKF;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.05; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        //TODO: This must be tuned to specific robot
        public static final double driveKS = (0.082009 / 12); //original (0.32 /12) | grabbed 0.082009 from SISID
        public static final double driveKV = (2.3358 / 12); //original (1.51 /12) | grabbed 2.3358 from SISID
        public static final double driveKA = (0.18379 / 12); //original (0.27 /12) | grabbed 0.18379 from SISID

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 0.3; //was 4.5 meters per second
        public static final double desiredAccelerationTime = 2; //in seconds
        public static final double maxAcceleration = 0; //meters per second
        /** Radians per Second */
        public static final double maxAngularVelocity = 3.0; //was 10.0

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 6;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(118.03);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(2.19);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(2.28);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 9;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(245.65);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 1;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        /* Pathplanner Speed Constants */
        public static final double kPathMaxAcceleration = 1.0; //1.0 was default
        public static final double kPathMaxVelocity = 1.0; //1.0 was default   0.3
    

        /* Auto Controllers
         * Have to feed fairly large values to see signfigant change.
         * Values appear to be affecting how quickly the motors can change. (maybe)
         */
        public static final double kPXController = 1.0; //1.0 was default
        public static final double kPYController = 0.27; //1.0 was default   0.3
        
        // theta controller constants
        public static final double kIThetaController = 0.0; // 0.0 is default
        public static final double kPThetaController = 0.25; //1.0 was default  0.25 - looks good.

        /* pathplanner controller PIDs */
        public static final double kPPathXController = 0.0; //0.05
        public static final double kPPathYController = 0.0;
        public static final double kPPathThetaController = 0.0;
        public static final double kIPathThetaController = 0.0;        
        
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
    public static final class PIDValues {
        public static final double frontRightTurnP = 0.0; 
        public static final double frontRightTurnI = 0.0; 
        public static final double frontRightTurnD = 0.0; 
        public static final double frontRightDriveP = 0.0; 
        public static final double frontRightDriveI = 0.0; 
        public static final double frontRightDriveD = 0.0; 
        public static final double backLeftTurnP = 0.0; 
        public static final double backLeftTurnI = 0.0; 
        public static final double backLeftTurnD = 0.0; 
        public static final double backLeftDriveP = 0.0; 
        public static final double backLeftDriveI = 0.0; 
        public static final double backLeftDriveD = 0.0; 
        public static final double elevatorOneP = 0.02;
        public static final double elevatorOneI = 0.0;
        public static final double elevatorOneD = 0.0;
        public static final double elevatorTwoP = 0.5;
        public static final double elevatorTwoI = 0.0;
        public static final double elevatorTwoD = 0.0;
        public static final double sliderP = 0.02;
        public static final double sliderI = 0.0;
        public static final double sliderD = 0.0;
        public static final double wristP = 0.02; 
        public static final double wristI = 0.0; 
        public static final double wristD = 0.0; 
        public static final double handP = 0.0;
        public static final double handI = 0.0;
        public static final double handD = 0.0;
    }

    public static class CANPorts{
        /*public static final int frontRightTurn = 0;
        public static final int frontRightDrive = 0;
        public static final int frontLeftTurn = 0;
        public static final int frontLeftDrive = 0;
        public static final int backRightTurn = 0; 
        public static final int backRightDrive = 0; 
        public static final int backLeftTurn = 0; 
        public static final int backLeftDrive = 0;*/ 
        public static final int elevatorLeft = 13; 
        public static final int elevatorRight = 2;
        public static final int frontRightDrive = 23;
        public static final int rearRightDrive = 1;
        public static final int frontLeftDrive = 15;
        public static final int rearLeftDrive = 14;
        public static final int slider = 30; 
        public static final int wrist = 4; 
        public static final int hand = 12; 
    }

    public static class ButtonMap{
        public static class Pilot{
            
        }
        public static class Copilot{
        //   public static final int shoot = XboxController.Button.kA.value;
        //   public static final int intake = XboxController.Button.kB.value;
          public static final int elevatorLow = XboxController.Button.kLeftBumper.value;
          public static final int elevatorMid = XboxController.Button.kBack.value;
          public static final int elevatorHigh = XboxController.Button.kRightBumper.value;
          public static final int gyro = XboxController.Button.kX.value;
          public static final int zeroGyro = XboxController.Button.kX.value;
          public static final int sliderIn = XboxController.Button.kStart.value;
          public static final int sliderOut = XboxController.Button.kB.value;
        }

    }
    public static class Subsys{
        public static final double elevatorLow = 0.0;
        public static final double elevatorMid = 20.0;
        public static final double elevatorHigh = 40.0;
        public static final double sliderIn = 0.0;
        public static final double sliderOut = 150.0;
        public static final double wristLow = 0.0;
        public static final double wristHigh = 20.0;
    }

    public static class RampConstants{
        public static final double P = 0.0; // P as in PID
        public static final double maxRampSpeed = 0.0; // percentage
    }
}
