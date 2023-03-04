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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoCenter;
import frc.robot.commands.TeleopSwerve;
import frc.robot.paths.AApath;
import frc.robot.paths.ExamplePath;
import frc.robot.paths.JustDriveAuto;
import frc.robot.paths.JustRamp;
import frc.robot.paths.OutandInPath;
import frc.robot.paths.snakePath;
import frc.robot.paths.pathGroups.Score1Final;
import frc.robot.paths.pathGroups.Score1RampFinal;
import frc.robot.paths.pathGroups.Score2Final;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    CommandBase selectedAuto;

    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    //private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton autoTrackLeft = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
    private final JoystickButton autoTrackRight = new JoystickButton(driver, XboxController.Button.kRightBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Limelight s_Limelight = new Limelight();

    /* Smartdashboard Choosers */
    private SendableChooser<String> autoChooser;
    

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {        

        //Constants.AutoConstants.smartDashboardAutoPIDs();
        //Constants.Swerve.smartDashboardDrivePIDs();

        /* Autonomous chooser */
        autoChooser = new SendableChooser<String>();
        SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);
        autoChooser.setDefaultOption("auto1", "3");
        autoChooser.addOption("examplePath", "1");
        autoChooser.addOption("AAPath", "2");
        autoChooser.addOption("snakePath", "3");
        autoChooser.addOption("OutandInPath", "4");

        // auto chooser final
        autoChooser.addOption("Do Nothing", "0");
        autoChooser.addOption("Just Drive", "5"); //make it case 1 on final
        autoChooser.addOption("Score 1", "6"); //make it case 2 on final
        autoChooser.addOption("Go on Ramp", "7"); //make it case 3 on final.
        autoChooser.addOption("Score 2", "8");
        autoChooser.addOption("Score 1, Get on Ramp", "9");



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
        autoTrackLeft.whileTrue(new AutoCenter(s_Swerve, s_Limelight, false));
        autoTrackRight.whileTrue(new AutoCenter(s_Swerve, s_Limelight, true));
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
                selectedAuto = ExamplePath.followTrajectoryCommand(true, s_Swerve);
                break;

                case "2":
                selectedAuto = AApath.followTrajectoryCommand(true, s_Swerve);
                break;

                case "3":
                selectedAuto = snakePath.followTrajectoryCommand(true, s_Swerve);
                break;

                case "4":
                selectedAuto = OutandInPath.followTrajectoryCommand(true, s_Swerve);
                break;

                case "5":
                selectedAuto = JustDriveAuto.followTrajectoryCommand(true, s_Swerve);
                break;

                case "6":
                selectedAuto = Score1Final.followTrajectoryCommand(s_Swerve);
                break;

                case "7":
                selectedAuto = JustRamp.followTrajectoryCommand(true, s_Swerve);
                break;

                case "8":
                selectedAuto = Score2Final.followTrajectoryCommand(s_Swerve);
                break;

                case "9":
                selectedAuto = Score1RampFinal.followTrajectoryCommand(s_Swerve);
                break;
            }

        return selectedAuto;
    }
}
