package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Swerve;
import frc.robot.commands.WristFunctionality.*;
import frc.robot.commands.DriveFunctionality.*;
import frc.robot.commands.Pause;
import frc.robot.subsystems.*;

public class Score1RampCube extends SequentialCommandGroup {
    private static Swerve s_Swerve;
    private static Hand s_Hand;
    

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;
        s_Hand = hand;

        return new SequentialCommandGroup(
            new IntakeAuto(s_Hand),
            new Drive(s_Swerve, -0.40, 0, 3, 0.0),
            new Pause(1.25),
            new DriveOnRamp(s_Swerve, true),
            new GyroStabalize(s_Swerve)
        );
    }
}
