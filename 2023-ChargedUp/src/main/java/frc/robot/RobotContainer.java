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
import frc.robot.commands.TeleopSwerve;
import frc.robot.paths.JustDriveAuto;
import frc.robot.paths.pathGroups.Score1Ramp;
import frc.robot.paths.pathGroups.rightGroups.*;
import frc.robot.paths.pathGroups.leftGroups.*;
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
    private final JoystickButton robotCentric = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);

    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();

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
                selectedAuto = JustDriveAuto.followTrajectoryCommand(true, s_Swerve);
                break;

                case "2":
                selectedAuto = RScore1.followTrajectoryCommand(s_Swerve);
                break;

                case "3":
                selectedAuto = LScore1.followTrajectoryCommand(s_Swerve); 
                break;

                case "4":
                selectedAuto = Score1Ramp.followTrajectoryCommand(s_Swerve);
                break;

                case "5":
                selectedAuto = RScore2.followTrajectoryCommand(s_Swerve);
                break;

                case "6":
                selectedAuto = LScore2.followTrajectoryCommand(s_Swerve);
                break;
                
                case "7":
                selectedAuto = RScore2Ramp.followTrajectoryCommand(s_Swerve);
                break;

                case "8":
                selectedAuto = LScore2Ramp.followTrajectoryCommand(s_Swerve);
                break;

                case "9":
                selectedAuto = RScore3.followTrajectoryCommand(s_Swerve);
                break;

                case "10":
                selectedAuto = LScore3.followTrajectoryCommand(s_Swerve);
                break;

                case "11":
                selectedAuto = RUltimate.followTrajectoryCommand(s_Swerve);
                break;

                case "12":
                selectedAuto = LUltimate.followTrajectoryCommand(s_Swerve);
                break;
            }

        return selectedAuto;
    }
}
