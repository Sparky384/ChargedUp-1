package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.Subsys;
import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.WristFunctionality.Intake;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private Elevator elevator = new Elevator();
    private Hand hand = new Hand();
    private Slider slider = new Slider();
    private Wrist wrist = new Wrist();

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton shoot = new JoystickButton(driver, Constants.ButtonMap.Copilot.shoot);
    private final JoystickButton intake = new JoystickButton(driver, Constants.ButtonMap.Copilot.intake);
    private final JoystickButton elevatorLow = new JoystickButton(driver, Constants.ButtonMap.Copilot.elevatorLow);
    private final JoystickButton elevatorMid = new JoystickButton(driver, Constants.ButtonMap.Copilot.elevatorMid);
    private final JoystickButton elevatorHigh = new JoystickButton(driver, Constants.ButtonMap.Copilot.elevatorHigh);
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton gyrJoystickButton = new JoystickButton(driver, Constants.ButtonMap.Copilot.gyro);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
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
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        shoot.whileTrue(new Shoot(hand));
        intake.whileTrue(new Intake(hand));
        elevatorLow.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorLow));
        elevatorMid.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorMid));
        elevatorHigh.onTrue(new MoveElevator(elevator, Constants.Subsys.elevatorHigh));
        // run DriveOnRamp THEN run GyroStabalize
        gyrJoystickButton.toggleOnTrue(new SequentialCommandGroup(
            new DriveOnRamp(s_Swerve),
            new GyroStabalize(s_Swerve)));
        
        /*
         * This is example command group, each command will run at the same time
         * Button.onTrue(new ParallelCommandGroup(new MoveElevator(elevator, 0),
         * new MoveSlider(slider, 0),
         * new MoveWrist(wrist, 0)));
         * 0 in this case references parameter i.e. elevatorLow
         */
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new exampleAuto(s_Swerve);
    }
}
