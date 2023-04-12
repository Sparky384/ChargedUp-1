package frc.robot;

import com.ctre.phoenix.platform.can.AutocacheState;

import edu.wpi.first.wpilibj.DriverStation;
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

import frc.robot.autonomous.*;
import frc.robot.commands.*;
import frc.robot.commands.CommandGroups.ToFeeder;
import frc.robot.commands.CommandGroups.ToGround;
import frc.robot.commands.CommandGroups.ToHigh;
import frc.robot.commands.CommandGroups.ToLow;
import frc.robot.commands.CommandGroups.ToMid;
import frc.robot.commands.DriveFunctionality.DriveOverRamp;
import frc.robot.commands.DriveFunctionality.GyroStabalize;
import frc.robot.commands.DriveFunctionality.TeleopSwerve;
import frc.robot.commands.ElevatorFunctionality.ManualElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.SliderFunctionality.SingleSlide;
import frc.robot.commands.WristFunctionality.Intake;
import frc.robot.commands.WristFunctionality.Outtake;
import frc.robot.commands.WristFunctionality.ShootCube;
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
    private double speedChangerTurn = 1.0; //is percent
    private double speedChanger = 1.0; //is percent

    /* Final Robot Buttons */
    //Pilot
    private final Trigger intakeBtn = pilot.rightTrigger();
    private final Trigger outtakeBtn = pilot.leftTrigger();
    private final Trigger shootCubeBtn = pilot.rightBumper();
    private final Trigger slowBtn = pilot.leftBumper();
    private final Trigger rampBtn = pilot.a();
    private final Trigger rampBtn2 = pilot.b();
    private final Trigger zeroGyroBtn = pilot.start();
    private final Trigger wristLowBtn = pilot.povDown();
    private final Trigger wristMidBtn = pilot.povRight();
    private final Trigger wristHighBtn = pilot.povUp();

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
    
    /* Smartdashboard Choosers */
    private SendableChooser<autoChooserEnum> autoChooser;
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    enum autoChooserEnum{
        DO_NOTHING,
        CUBE_LEFT,
        CUBE_RIGHT,
        CONE,
        RAMP_CONE,
        RAMP_CUBE,
        RAMP_CUBE_PICKUP,
        BLUE_2SCORE_RIGHT,
        BLUE_2SCORE_LEFT,
        BLUE_3SCORE_RIGHT,
        BLUE_3SCORE_LEFT;
    }


    public RobotContainer() {   
             
        /* Autonomous chooser */
        autoChooser = new SendableChooser<>();
        SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);


        // auto chooser final
        autoChooser.setDefaultOption("Do Nothing", autoChooserEnum.DO_NOTHING);
        autoChooser.addOption("Do Nothing", autoChooserEnum.DO_NOTHING);
        autoChooser.addOption("CubeLeft", autoChooserEnum.CUBE_LEFT);
        autoChooser.addOption("CubeRight", autoChooserEnum.CUBE_RIGHT);
        autoChooser.addOption("Cone", autoChooserEnum.CONE);
        autoChooser.addOption("Ramp Cone", autoChooserEnum.RAMP_CONE);
        autoChooser.addOption("Ramp Cube", autoChooserEnum.RAMP_CUBE);
        autoChooser.addOption("Ramp Cube and Pickup **TESTING**", autoChooserEnum.RAMP_CUBE_PICKUP);
        autoChooser.addOption("2 Score Right", autoChooserEnum.BLUE_2SCORE_RIGHT);
        autoChooser.addOption("2 Score Left", autoChooserEnum.BLUE_2SCORE_LEFT);
        autoChooser.addOption("3 Score Right **TESTING**", autoChooserEnum.BLUE_3SCORE_RIGHT);
        autoChooser.addOption("3 Score Left **TESTING**", autoChooserEnum.BLUE_3SCORE_LEFT);

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
        shootCubeBtn.whileTrue(new ShootCube(hand));
        slowBtn.whileTrue(new InstantCommand(() -> {speedChangerTurn = Constants.SlowBtnSpeedTurn; speedChanger = Constants.SlowBtnSpeed;}));
        slowBtn.whileFalse(new InstantCommand(() -> {speedChangerTurn = 1.0; speedChanger = 1.0;}));
        
        rampBtn.whileTrue(new SequentialCommandGroup(
            new DriveOverRamp(swerve, true),
            new GyroStabalize(swerve))
        );
        
        rampBtn2.onTrue(new DriveOverRamp(swerve, false));
        wristHighBtn.onTrue(wrist.wristMotionMagic(Constants.Subsys.wristHigh));
        wristMidBtn.onTrue(wrist.wristMotionMagic(Constants.Subsys.wristMid));
        wristLowBtn.onTrue(wrist.wristMotionMagic(Constants.Subsys.wristGround));
        //setAngle.onTrue(new InstantCommand(() -> swerve.setAngle(180))); was testing command keeping in case we want to test it again.

        //copilot
        sliderInBtn.onTrue(new MoveSlider(slider, Constants.Subsys.sliderIn, 0.0));
        sliderOutBtn.onTrue(new MoveSlider(slider, Constants.Subsys.sliderOut, 0.0));
        stopIntakeBtn.onTrue(new StopIntake(hand));
        
        stowBtn.onTrue(new ParallelCommandGroup(
            wrist.wristMotionMagic(Constants.Subsys.wristHigh),
            new MoveSlider(slider, Constants.Subsys.sliderIn, 0.0),
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
            swerve.setAngle(180);

            // Chooser for different autonomous functions.
            switch(autoChooser.getSelected()) {

                case DO_NOTHING:
                selectedAuto = null;
                break;
                
                case CUBE_LEFT:
                selectedAuto = LeftCube1.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case CUBE_RIGHT:
                selectedAuto = RightCube.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
                
                case CONE:
                selectedAuto = Cone.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
                
                case RAMP_CONE:
                selectedAuto = ScoreRamp.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case RAMP_CUBE:
                selectedAuto = Score1RampCube.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
                
                case RAMP_CUBE_PICKUP:
                selectedAuto = Score1RampCubePickup.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case BLUE_2SCORE_RIGHT:
                selectedAuto = Blue2ScoreRightManual.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case BLUE_2SCORE_LEFT:
                selectedAuto = Blue2ScoreLeftManual.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case BLUE_3SCORE_RIGHT:
                selectedAuto = Blue3ScoreRightManual.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;

                case BLUE_3SCORE_LEFT:
                selectedAuto = Blue3ScoreLeftManual.followTrajectoryCommand(swerve, elevator, slider, wrist, hand);
                break;
            }

        return selectedAuto;
    }

    public void periodic()
    {
        //pilot controlling swerve
        swerve.setDefaultCommand(
            new TeleopSwerve(
                swerve, 
                () -> pilot.getLeftY() * speedChanger, 
                () -> pilot.getLeftX() * speedChanger, 
                () -> pilot.getRightX() * speedChangerTurn, 
                () -> Constants.Swerve.robotcentric //pass in true for robotcentric false for fieldcentric
            )
        );
    }
}
