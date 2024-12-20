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
import edu.wpi.first.wpilibj.smartdashboard.*;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double wristThreshold = 1.0;
    public static final double sliderThreshold = 1.0;
    public static final double elevatorThreshold = 0.2;
    public static final double rampThreshold = 9.0; // in degrees

    public static final double elevatorSpeedLimit = 0.3;
    public static final double elevatorDownSpeedLimit = 0.0;
    public static final double SlowBtnSpeedTurn = 0.15; //in percent
    public static final double SlowBtnSpeed = 0.20;

    

    public static final class Swerve {
        public static final int pigeonID = 20;
        public static final double slowDriveAmount = .5; //change depending on amount we want to slow down by when holding button.
        public static final boolean robotcentric = false; //change depending on if robotcentric wanted/not wanted.
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
        public static final double openLoopRamp = 0.25; //was 0.25
        public static final double closedLoopRamp = 0.0; //was 0.0

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

        // for changing PIDs for Drive with Smartdashboard. remove "final" from variable. Comment ones not being used.
        /*public static void smartDashboardDrivePIDs() {
            driveKP = SmartDashboard.getNumber("DriveKP", driveKP);
            driveKI = SmartDashboard.getNumber("DriveKI", driveKI);
            driveKD = SmartDashboard.getNumber("DriveKD", driveKD);
            driveKF = SmartDashboard.getNumber("DriveKF", driveKF);
        }*/

        /* Drive Motor Characterization Values 
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE */
        //TODO: This must be tuned to specific robot
        public static final double driveKS = (0.18882 / 12); //original (0.32 /12) | grabbed 0.082009 from SISID
        public static final double driveKV = (2.6515 / 12); //original (1.51 /12) | grabbed 2.3358 from SISID
        public static final double driveKA = (0.37384 / 12); //original (0.27 /12) | grabbed 0.18379 from SISID

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 0.3; //was 4.5 meters per second
        public static final double desiredAccelerationTime = 2; //in seconds
        public static final double maxAcceleration = 0; //meters per second
        /** Radians per Second */
        public static final double maxAngularVelocity = 1.0; //original 10.0 was 3.0

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Module Specific Constants */
        
        /* Front Left Module - Module 0 */
        public static final class Mod0 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 36; //not sure yet
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(298.564); //298.213
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 37; //not sure yet
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(181.934); //.879
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 12;
            public static final int angleMotorID = 13;
            public static final int canCoderID = 34; //not sure yet
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(61.348); //63.105
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { //TODO: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 35; //not sure yet
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(66.006);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //TODO: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 1.0;
        public static final double kMaxAccelerationMetersPerSecondSquared = 1.0;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        /* Manual Auto Constants */
        public static final double DriveToPositionThreshold = Units.inchesToMeters(1.0);
        public static final double DriveToPositionXP = 0.634; //0.6
        public static final double DriveToPositionYP = 0.634; //0.6
        public static final double DriveToPositionXI = 0.0081; //.002
        public static final double DriveToPositionYI = 0.0081; //.002
        public static final double DriveToPositionXD = 0;
        public static final double DriveToPositionYD = 0;
        public static final double TurnToP = 0.00827;

        /* Pathplanner Speed Constants */
        public static final double kPathMaxAcceleration = 0.325; //1.0 was default
        public static final double kPathMaxVelocity = 0.325; //1.0 was default   0.3

        /* Intake and Outtake timers. */
        public static final double kAutoShootTimer = 0.5; //timer in seconds
        public static final double angleThreshold = 0.65;


    

        /* Auto Controllers
         * Have to feed fairly large values to see signfigant change.
         * Values appear to be affecting how quickly the motors can change. (maybe)
         */
        /* pathplanner controller PIDs. Make sure these have "final" in them when not tuning. */
        public static final double kPPathXController = 1.0; //1.0
        public static final double kPPathYController = 1.0; //0.23

        public static final double kPPathThetaController = 0.05; //1.575  10
        public static final double kIPathThetaController = 0.0; //0.0

        // for changing PIDs for Auto with Smartdashboard. remove "final" from variable. Comment ones not being used.
        /*public static void smartDashboardAutoPIDs() {
            kPPathThetaController = SmartDashboard.getNumber("Theta P", kPPathThetaController);
            kIPathThetaController = SmartDashboard.getNumber("Theta I", kIPathThetaController);
            kPPathXController = SmartDashboard.getNumber("Path X", kPPathXController);
            kPPathYController = SmartDashboard.getNumber("Path Y", kPPathYController);
        }*/
    
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final class PathplannerConstraints{
            public static final double Score1DriveOutPPathX = 1.0;
            public static final double Score1DriveOutPPathY = 0.23;
            public static final double Score1DriveOutPPathTheta = 1.0;

        }
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

        public static final double elevatorUpP = 0.090; //.08
        public static final double elevatorUpI = 0.0;
        public static final double elevatorUpD = 0.0;
        public static final double elevatorUpF = 0.05863; //FF was .06687
        public static final double elevatorDownP = 0.0;
        public static final double elevatorDownI = 0.0;
        public static final double elevatorDownD = 0.0;

        public static final double sliderP = 0.295;
        public static final double sliderI = 0.001;
        public static final double sliderIZone = 0.3;
        public static final double sliderD = 0.0;

        public static final double wristP = 1.2; //1.2 after recent change: 1.7
        public static final double wristI = 0.00005; //0.00005 before recent change: 0.0
        public static final double wristIZone = 0.02; //0.02 before recent change: 0.0
        public static final double wristD = 3.2; //3.2 after recent change: 3.3

        public static final double handP = 0.0;
        public static final double handI = 0.0;
        public static final double handD = 0.0;

        public static final double elevatorMaxSpeed = 0.2; 
        public static final double elevatorMinSpeed = -0.1; 
    }

    public static class ConversionValues {
        public static final double sliderConversionFunction = .798;
        public static final double elevatorConversionFunction = 0.000356;
        public static final double wristConversionFunction = 0.0; //not final.
    }

    public static class CANPorts{
        public static final int elevatorLeft = 16; // don't have yet 
        public static final int elevatorRight = 0; //final
        public static final int slider = 19; //final 19
        public static final int wristMotor = 18; //final
        public static final int wristCancoder = 2;
        public static final int hand = 17; //final 17
    }

    public static class Subsys{
        public static final double elevatorLow = 50;
        public static final double elevatorMid = 50; //in counts
        public static final double elevatorFeederStation = 59800;
        public static final double elevatorHigh = 89800; //in counts
        public static final double sliderIn = 0.05;
        public static final double sliderOut = 10.75; //final in inches
        public static final double wristGround = -14.184; //should be in degrees.
        public static final double wristGroundAuto = -15.2; //should be in degrees.
        public static final double wristLow = 14.941; //should be in degrees.
        public static final double wristMid = 38.408; //should be in degrees.
        public static final double wristFeederStation = 17.900;
        public static final double wristHigh = 90.0;
        public static final double wristHighGoal = 3.0; //should be in degrees.
        public static final double wristAbsEncoderOffset = 0.0;
        /* Motion Magic Constants */
        public static final int timeOutMs = 10;
        public static final double wristUpperLimit = -185.0; //this is in encoder counts from motion magic
        public static final double wristLowerLimit = 1204; //this is in encoder counts from motion magic
        public static final double wristThreshold = 2.0; //not final
        public static final double elevatorUpperLimit = 90370; //final
        public static final double elevatorLowerLimit = 0.0; //final
        public static final double elevatorThreshold = 500; //final (maybe)
        public static final double elevatorArbitraryFeedForward = 0.07000;
        public static final double wristArbitraryFeedForward = 0.0;

        //Wrist Feedforward Constants
        public static final double wristLengthMain = 0.0;
        public static final double wristWeightMain = 0.0;
        public static final double wristWeightBackup = 0.0;
        public static final double wristLengthBackup = 0.0;

        public static final double wristMotorOhms = 0.0;
        public static final double wristMotorTorque = 41.5;
        public static final double wristMotorStallCurrent = 257;

        public static final double wristMotorGearbox = 0.0;



    }

    public static class RampConstants{
        public static final double P = 0.0391; // P as in PID, 385 395
        public static final double maxRampSpeed = 0.25; // percentage
    }

    public static class LimelightConstants {

        public static class LimelightCameras {
            public static final String LIME1 = "limelight-local"; //setup as goal camera for now
            public static final String LIME2 = "limelight-limetwo"; //setup as element camera for now

        }

        public static enum LimelightPipelines
        {
            HI_GOAL,
            LOW_GOAL,
            CONE, 
            CUBE
        }

        public static final double limelightDeadband = 0.8;
    }

}
