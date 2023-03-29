package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.commands.DriveFunctionality.Drive;
import frc.robot.commands.WristFunctionality.*;

public class LeftCube1 extends SequentialCommandGroup{
    private static Swerve s_Swerve;
    private static Hand s_Hand;

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;
        s_Hand = hand;

        return new SequentialCommandGroup(
            //ToHigh.getToHigh(s_Elevator, s_Slider, wrist),
            new IntakeAuto(s_Hand), //assumes we start with cube so this is outtake for cube
            //new MoveSlider(s_Slider, Constants.Subsys.sliderIn, 0),
            //Stow.getStowCommand(s_Elevator, s_Slider, wrist),
            new Drive(s_Swerve, -0.1, 0.2, 0.5),
            new Drive(s_Swerve, -0.4, 0.0, 2.3)
        );
    }
}
