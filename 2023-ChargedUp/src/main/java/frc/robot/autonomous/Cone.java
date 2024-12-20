package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.Constants;
import frc.robot.commands.DriveFunctionality.Drive;
import frc.robot.commands.CommandGroups.*;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.*;

public class Cone extends SequentialCommandGroup{
    private static Swerve s_Swerve;
    private static Elevator s_Elevator;
    private static Slider s_Slider;
    private static Wrist s_Wrist;
    private static Hand s_Hand;

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;
        s_Elevator = elevator;
        s_Slider = slider;
        s_Wrist = wrist;
        s_Hand = hand;

        return new SequentialCommandGroup(
            ToHigh.getToHigh(s_Elevator, s_Slider, s_Wrist),
            new OuttakeAuto(s_Hand, Constants.AutoConstants.kAutoShootTimer), //assumes we start with cube so this is outtake for cube
            new MoveSlider(s_Slider, Constants.Subsys.sliderIn, 0),
            Stow.getStowCommand(s_Elevator, s_Slider, s_Wrist),
            new Drive(s_Swerve, -0.4, 0.0, 2.45)
        );
    }
}
