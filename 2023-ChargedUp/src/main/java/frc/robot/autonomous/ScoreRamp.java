package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;
import frc.robot.commands.WristFunctionality.*;
import frc.robot.commands.DriveFunctionality.*;
import frc.robot.commands.CommandGroups.*;
import frc.robot.subsystems.*;

public class ScoreRamp extends SequentialCommandGroup {
    private static Swerve s_Swerve;
    private static Elevator s_Elevator;
    private static Slider s_Slider;
    private static Hand s_Hand;
    

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;
        s_Elevator = elevator;
        s_Slider = slider;
        s_Hand = hand;

        return new SequentialCommandGroup(
            ToHigh.getToHigh(s_Elevator, s_Slider, wrist),
            new OuttakeAuto(s_Hand, Constants.AutoConstants.kAutoShootTimer),
            Stow.getStowCommand(s_Elevator, s_Slider, wrist),
            new DriveOnRamp(s_Swerve, false),
            new GyroStabalize(s_Swerve)
        );
    }
}