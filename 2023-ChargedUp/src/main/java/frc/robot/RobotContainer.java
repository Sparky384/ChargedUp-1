package frc.robot;

import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCenter;
import frc.robot.commands.TeleopSwerve;
import frc.robot.paths.JustDriveAuto;
import frc.robot.paths.pathGroups.Score1Ramp;
import frc.robot.paths.pathGroups.rightGroups.*;
import frc.robot.paths.pathGroups.leftGroups.*;
import frc.robot.paths.JustRamp;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.commands.*;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.Intake;
import frc.robot.commands.WristFunctionality.Outtake;
import frc.robot.commands.WristFunctionality.RotateWrist;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick pilot = new Joystick(0);
    private final Joystick copilot = new Joystick(1);
    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private Elevator elevator = new Elevator();
    private Limelight s_Limelight = new Limelight();
    private Hand hand = new Hand();
    private Slider slider = new Slider();
    private Wrist wrist = new Wrist();
    CommandBase selectedAuto;

    /* Pilot Joystick Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Copilot Joystick Controls */
    private final int elevatorJoystick = XboxController.Axis.kLeftY.value;

    /* Pilot Buttons */
    // private final JoystickButton shoot = new JoystickButton(pilot, Constants.ButtonMap.Copilot.shoot);
    // private final JoystickButton intake = new JoystickButton(pilot, Constants.ButtonMap.Copilot.intake);
    private final JoystickButton elevatorLow = new JoystickButton(pilot, Constants.ButtonMap.Copilot.elevatorLow);
    private final JoystickButton elevatorMid = new JoystickButton(pilot, Constants.ButtonMap.Copilot.elevatorMid);
    private final JoystickButton elevatorHigh = new JoystickButton(pilot, Constants.ButtonMap.Copilot.elevatorHigh);
    private final JoystickButton robotCentric = new JoystickButton(pilot, XboxController.Button.kLeftBumper.value);
    private final JoystickButton gyrJoystickButton = new JoystickButton(pilot, Constants.ButtonMap.Pilot.driveOnRampSequence);
    private final JoystickButton sliderIn = new JoystickButton(pilot, Constants.ButtonMap.Copilot.sliderIn);
    private final JoystickButton sliderOut = new JoystickButton(pilot, Constants.ButtonMap.Copilot.sliderOut);

    /* Final Robot Buttons */
    //Pilot
    //private final JoystickButton toggleLED = new JoystickButton(pilot, ); changes LED don't have limelight on this version yet
    private final JoystickButton driveOnRampSequence = new JoystickButton(pilot, Constants.ButtonMap.Pilot.driveOnRampSequence);
    private final JoystickButton pickupObject = new JoystickButton(pilot, Constants.ButtonMap.Pilot.pickupObject);
    private final JoystickButton dropObject = new JoystickButton(pilot, Constants.ButtonMap.Pilot.dropObject);
    private final JoystickButton slowDrive = new JoystickButton(pilot, Constants.ButtonMap.Pilot.slowDrive);
    private final JoystickButton autoTrackLeft = new JoystickButton(pilot, Constants.ButtonMap.Pilot.pickupObject);
    private final JoystickButton autoTrackRight = new JoystickButton(pilot, Constants.ButtonMap.Pilot.pickupObject);
    //copilot
    //private final JoystickButton lowGoalLimelight = new J

    /* Smartdashboard Choosers */
    private SendableChooser<String> autoChooser;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {        

        //Constants.AutoConstants.smartDashboardAutoPIDs();
        //Constants.Swerve.smartDashboardDrivePIDs();

        /* Autonomous chooser */
        autoChooser = new SendableChooser<String>();
        SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);

        // auto chooser final
        autoChooser.setDefaultOption("Do Nothing", "0");
        autoChooser.addOption("Do Nothing", "0");
        autoChooser.addOption("Just Drive", "1");
        autoChooser.addOption("RScore 1", "2");
        autoChooser.addOption("LScore 1", "3");
        autoChooser.addOption("Score 1 Ramp", "4");
        autoChooser.addOption("RScore 2", "5");
        autoChooser.addOption("LScore 2", "6");
        autoChooser.addOption("RScore 2 Ramp", "7");
        autoChooser.addOption("LScore 2 Ramp", "8");
        autoChooser.addOption("RScore 3", "9");
        autoChooser.addOption("LScore 3", "10");
        autoChooser.addOption("RScore 3 Ramp", "11");
        autoChooser.addOption("LScore 3 Ramp", "12");




        //pilot controlling swerve
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> -pilot.getRawAxis(translationAxis), 
                () -> -pilot.getRawAxis(strafeAxis), 
                () -> -pilot.getRawAxis(rotationAxis), 
                () -> robotCentric.getAsBoolean()
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* final configurations */
        // run DriveOnRamp THEN run GyroStabalize
        //driveOnRampSequence.toggleOnTrue(new SequentialCommandGroup(
            // new DriveOnRamp(swerve, false),
            // new GyroStabalize(swerve)));
        
        /* Driver Buttons */
        // zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        // shoot.whileTrue(new Shoot(hand));
        // intake.whileTrue(new Intake(hand));
        elevatorLow.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorLow));
        elevatorMid.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorMid));
        elevatorHigh.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorHigh));
        //sliderIn.onTrue(new MoveSlider(slider, Constants.Subsys.sliderIn));
        //sliderOut.onTrue(new MoveSlider(slider, Constants.Subsys.sliderOut));
        //sliderIn.onTrue(new RotateWrist(wrist, Constants.Subsys.wristLow));
        //sliderOut.onTrue(new RotateWrist(wrist, Constants.Subsys.wristHigh));
        sliderIn.whileTrue(new Intake(hand));
        sliderOut.whileTrue(new Outtake(hand));
        // run DriveOnRamp THEN run GyroStabalize
        driveOnRampSequence.toggleOnTrue(new SequentialCommandGroup(
            new DriveOnRamp(swerve, false),
            new GyroStabalize(swerve)));
        
        /*
         * This is example command group, each command will run at the same time
         * Button.onTrue(new ParallelCommandGroup(new MoveElevator(elevator, 0),
         * new MoveSlider(slider, 0),
         * new MoveWrist(wrist, 0)));
         * 0 in this case references parameter i.e. elevatorLow
         */
        autoTrackLeft.whileTrue(new AutoCenter(swerve, s_Limelight, false));
        autoTrackRight.whileTrue(new AutoCenter(swerve, s_Limelight, true));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {

            // Chooser for different autonomous functions.
            switch(autoChooser.getSelected()) {


                case "1":
                selectedAuto = JustDriveAuto.followTrajectoryCommand(true, swerve);
                break;

                case "2":
                selectedAuto = RScore1.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case "3":
                selectedAuto = LScore1.followTrajectoryCommand(swerve, elevator, slider, wrist, hand); 
                break;

                case "4":
                selectedAuto = Score1Ramp.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case "5":
                selectedAuto = RScore2.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case "6":
                selectedAuto = LScore2.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
                
                case "7":
                selectedAuto = RScore2Ramp.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case "8":
                selectedAuto = LScore2Ramp.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case "9":
                selectedAuto = RScore3.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case "10":
                selectedAuto = LScore3.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case "11":
                selectedAuto = RUltimate.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case "12":
                selectedAuto = LUltimate.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
            }

        return null;
    }
}
