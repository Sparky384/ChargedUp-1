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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoCenter;
import frc.robot.commands.TeleopSwerve;
import frc.robot.paths.JustDriveAuto;
import frc.robot.paths.pathGroups.Score1Ramp;
import frc.robot.paths.pathGroups.rightGroups.*;
import frc.robot.paths.pathGroups.leftGroups.*;
import frc.robot.paths.JustRamp;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

import frc.robot.commands.*;
import frc.robot.commands.ElevatorFunctionality.ManualElevator;
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
    private final CommandXboxController pilot = new CommandXboxController(0);
    private final CommandXboxController copilot = new CommandXboxController(1);
    /* Subsystems */
    private final Swerve swerve = new Swerve();
    private Elevator elevator = new Elevator();
    private Limelight s_Limelight = new Limelight();
    private Hand hand = new Hand();
    private Slider slider = new Slider();
    private Wrist wrist = new Wrist();
    CommandBase selectedAuto;

    /* Pilot Joystick Controls */
    private final double translationAxis = pilot.getLeftY();
    private final double strafeAxis = pilot.getLeftX();
    private final double rotationAxis = pilot.getRightX();

    /* Final Robot Buttons */
    //Pilot
    private final Trigger toggleLED = pilot.back(); //changes LED don't have limelight on this version yet
    private final Trigger driveOnRampSequence = pilot.start();
    private final Trigger pickupObject = pilot.a();
    private final Trigger dropObject = pilot.b();
    private final Trigger slowDrive = pilot.rightTrigger();
    private final Trigger autoTrackLeft = pilot.leftBumper();
    private final Trigger autoTrackRight = pilot.rightBumper();
    private final Trigger zeroGyro = pilot.povDown();
    
    //copilot
    private final Trigger lowGoalLimelight = copilot.leftStick();
    private final Trigger highGoalLimelight = copilot.b();
    private final Trigger coneLimelight = copilot.leftTrigger();
    private final Trigger cubeLimelight = copilot.rightTrigger();
    private final Trigger sliderIn = copilot.back();
    private final Trigger sliderOut = copilot.start();
    private final Trigger elevatorHigh = copilot.y();
    private final Trigger elevatorMid = copilot.x();
    private final Trigger elevatorLow = copilot.a();
    private final double elevatorX = copilot.getRightX();
    private final Trigger handIntake = copilot.leftTrigger();
    private final Trigger handOuttake = copilot.rightTrigger();
    private final Trigger wristHigh = copilot.povUp();
    private final Trigger wristMid = copilot.povRight();
    private final Trigger wristLow = copilot.povLeft();
    private final Trigger wristGround = copilot.povDown();

    /* Copilot Joystick Controls */
    private final double elevatorJoystick = copilot.getLeftX();

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
        if (slowDrive.getAsBoolean() == true){
            swerve.setDefaultCommand(
                new TeleopSwerve(
                    swerve, 
                    () -> -translationAxis * Constants.Swerve.slowDriveAmount, 
                    () -> -strafeAxis * Constants.Swerve.slowDriveAmount, 
                    () -> -rotationAxis * Constants.Swerve.slowDriveAmount, 
                    () -> Constants.Swerve.robotcentric //pass in true for robotcentric false for fieldcentric
                    )
            );
        } else {
            swerve.setDefaultCommand(
                new TeleopSwerve(
                    swerve, 
                    () -> -translationAxis, 
                    () -> -strafeAxis, 
                    () -> -rotationAxis, 
                    () -> Constants.Swerve.robotcentric //pass in true for robotcentric false for fieldcentric
                )
            );
        }

        // Copilot Manual Elevator

        elevator.setDefaultCommand(
            new ManualElevator(
                elevator,
                () -> elevatorX
            )
        );

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.}.
     */
    private void configureButtonBindings() {
        /* final configurations */
        //pilot
        toggleLED.onTrue(new InstantCommand(() -> s_Limelight.toggleLED()));
        pickupObject.onTrue(new ParallelCommandGroup(new Intake(hand), new RotateWrist(wrist, Constants.Subsys.wristLow))); //will change to Constants.Subsys.wristGround later
        dropObject.onTrue(new ParallelCommandGroup(new Outtake(hand), new RotateWrist(wrist, Constants.Subsys.wristLow))); //will likely have to change the height based on which node we're dropping the game piece in.
        zeroGyro.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        // run DriveOnRamp THEN run GyroStabalize
        driveOnRampSequence.toggleOnTrue(new SequentialCommandGroup(
            new DriveOnRamp(swerve, false),
            new GyroStabalize(swerve)));
        
        //copilot
        lowGoalLimelight.onTrue(new InstantCommand(() -> s_Limelight.switchProfile(Constants.LimelightPipelines.LOW_GOAL)));
        highGoalLimelight.onTrue(new InstantCommand(() -> s_Limelight.switchProfile(Constants.LimelightPipelines.HI_GOAL)));
        coneLimelight.onTrue(new InstantCommand(() -> s_Limelight.switchProfile(Constants.LimelightPipelines.CONE)));
        cubeLimelight.onTrue(new InstantCommand(() -> s_Limelight.switchProfile(Constants.LimelightPipelines.CUBE)));
        sliderIn.onTrue(new MoveSlider(slider, Constants.Subsys.sliderIn));
        sliderOut.onTrue(new MoveSlider(slider, Constants.Subsys.sliderOut));
        elevatorHigh.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorHigh));
        elevatorMid.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorMid));
        elevatorLow.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorLow));
        handIntake.onTrue(new Intake(hand));
        handOuttake.onTrue(new Outtake(hand));
        wristHigh.onTrue(new RotateWrist(wrist, Constants.Subsys.wristHigh));
        wristMid.onTrue(new RotateWrist(wrist, Constants.Subsys.wristMid));
        wristLow.onTrue(new RotateWrist(wrist, Constants.Subsys.wristLow));
        wristGround.onTrue(new RotateWrist(wrist, Constants.Subsys.wristGround));
        elevatorLow.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorLow));
        elevatorMid.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorMid));
        elevatorHigh.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorHigh));
        sliderIn.whileTrue(new Intake(hand));
        sliderOut.whileTrue(new Outtake(hand));
        
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

                case "0":
                selectedAuto = null;
                break;

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

        return selectedAuto;
    }
}
