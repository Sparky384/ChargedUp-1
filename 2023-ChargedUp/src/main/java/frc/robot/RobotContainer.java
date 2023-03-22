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
import frc.robot.paths.JustDriveAuto;
import frc.robot.paths.pathGroups.CubeScore1Ramp;
import frc.robot.paths.pathGroups.Score1DriveOut;
import frc.robot.paths.pathGroups.Score1Ramp;
import frc.robot.paths.pathGroups.Score1RampNew;
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
import frc.robot.commands.WristFunctionality.StopIntake;
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

    /* Final Robot Buttons */
    //Pilot
    private final Trigger intakeBtn = pilot.rightTrigger();
    private final Trigger outtakeBtn = pilot.leftTrigger();
    private final Trigger rampBtn = pilot.a();
    private final Trigger rampBtn2 = pilot.b();
    private final Trigger zeroGyroBtn = pilot.start();
    private final Trigger wristLowBtn = pilot.povDown();
    private final Trigger wristMidBtn = pilot.povRight();
    private final Trigger wristHighBtn = pilot.povUp();

    // mid; ele low sli in wrist = 38.408
    // ho; ele hi sli out wrist low
    //copilot
    private final Trigger stopIntakeBtn = copilot.rightBumper();
    private final Trigger stowBtn = copilot.leftBumper();
    private final Trigger toFeederBtn = copilot.x();
    private final Trigger toHighBtn = copilot.y();
    private final Trigger toMidBtn = copilot.b();
    private final Trigger toLowBtn = copilot.a();
    private final Trigger sliderOutBtn = copilot.povUp();
    private final Trigger sliderInBtn = copilot.povDown();
    private final Trigger toGroundBtn = copilot.povRight();
    private final Trigger elevatorHighBtn = copilot.start();
    private final Trigger elevatorLowBtn = copilot.back();
    private final double elevatorJoystick = copilot.getLeftX();

    /* Smartdashboard Choosers */
    private SendableChooser<String> autoChooser;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {        
        /* Autonomous chooser */
        autoChooser = new SendableChooser<String>();
        SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);

        // auto chooser final
        autoChooser.setDefaultOption("Do Nothing", "0");
        autoChooser.addOption("Do Nothing", "0");
        autoChooser.addOption("Just Drive", "1");
        autoChooser.addOption("RScore 1", "2");
        autoChooser.addOption("LScore 1", "3");
        autoChooser.addOption("Cone Score 1 Ramp", "4");
        autoChooser.addOption("RScore 2", "5");
        autoChooser.addOption("LScore 2", "6");
        autoChooser.addOption("Cone R Score 2", "7");
        autoChooser.addOption("Score 1 Ramp New", "8");
        autoChooser.addOption("Cube Score 1 Ramp", "9");
        autoChooser.addOption("Score and Drive", "10");

        //autoChooser.addOption("TEST ONLY*** Flip test", "7");

        //pilot controlling swerve
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> pilot.getLeftY(), 
                () -> pilot.getLeftX(), 
                () -> pilot.getRightX(), 
                () -> Constants.Swerve.robotcentric //pass in true for robotcentric false for fieldcentric
            )
        );

        // Copilot Manual Elevator

        /*elevator.setDefaultCommand(
            new ManualElevator(
                elevator,
                () -> copilot.getRightY()
            )
        );*/
        
        //wrist.setDefaultCommand(wrist.wristStick(() -> copilot.getLeftY())); //for manual wrist control with stick
        
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
        zeroGyroBtn.onTrue(new InstantCommand(() -> swerve.zeroGyro()));
        intakeBtn.whileTrue(new Intake(hand));
        outtakeBtn.whileTrue(new Outtake(hand));
        
        rampBtn.whileTrue(new SequentialCommandGroup(
            new DriveOverRamp(swerve, true),
            new GyroStabalize(swerve))
        );
        
        rampBtn2.onTrue(new DriveOverRamp(swerve, false));
        wristHighBtn.onTrue(wrist.wristMotionMagic(Constants.Subsys.wristHigh));
        wristMidBtn.onTrue(wrist.wristMotionMagic(Constants.Subsys.wristMid));
        wristLowBtn.onTrue(wrist.wristMotionMagic(Constants.Subsys.wristGround));

        //copilot
        sliderInBtn.onTrue(new MoveSlider(slider, Constants.Subsys.sliderIn));
        sliderOutBtn.onTrue(new MoveSlider(slider, Constants.Subsys.sliderOut));
        stopIntakeBtn.onTrue(new StopIntake(hand));
        
        stowBtn.onTrue(new ParallelCommandGroup(
            wrist.wristMotionMagic(Constants.Subsys.wristHigh),
            new MoveSlider(slider, Constants.Subsys.sliderIn),
            elevator.elevatorMotionMagic(Constants.Subsys.elevatorLow)
        ));
        
        toFeederBtn.onTrue(ToFeeder.getToFeeder(elevator, slider, wrist));
        toHighBtn.onTrue(ToHigh.getToHigh(elevator, slider, wrist));
        toMidBtn.onTrue(ToMid.getToMid(elevator, slider, wrist));
        toLowBtn.onTrue(ToLow.getToLow(elevator, slider, wrist));
        toGroundBtn.onTrue(ToGround.getToGround(elevator, slider, wrist));
        elevatorLowBtn.onTrue(elevator.elevatorMotionMagic(Constants.Subsys.elevatorLow));
        elevatorHighBtn.onTrue(elevator.elevatorMotionMagic(Constants.Subsys.elevatorHigh));
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
                selectedAuto = ConeRScore2.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
                
                case "8":
                selectedAuto = Score1RampNew.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
                
                case "9":
                selectedAuto = CubeScore1Ramp.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
                
                case "10":
                selectedAuto = Score1DriveOut.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
            }

        return selectedAuto;
    }

    public void periodic()
    {
    }
}
