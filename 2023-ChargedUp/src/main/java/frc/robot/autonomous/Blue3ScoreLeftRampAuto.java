package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.robot.commands.CommandGroups.Stow;
import frc.robot.commands.WristFunctionality.*;

import frc.robot.paths.leftPaths.Blue3ScoreLeftPickup;
import frc.robot.paths.leftPaths.Blue3ScoreLeftScore;
import frc.robot.paths.leftPaths.Blue3ScoreLeftRamp;

public class Blue3ScoreLeftRampAuto extends SequentialCommandGroup{
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
            //new IntakeAuto(s_Hand), //assumes we start with cube so this is outtake for cube

            Blue2ScoreLeft.followTrajectoryCommand(s_Swerve, s_Elevator, s_Slider, s_Wrist, s_Hand),
            
            Blue3ScoreLeftPickup.followTrajectoryCommand(true, s_Swerve), //goes to pickup cone.
            
            /* for final. Purpose is to put wrist down before we get to the end in an attempt to save as much time as possible.
            new ParallelCommandGroup(Blue2ScoreLeftPickup.followTrajectoryCommand(true, s_Swerve), 
            new SequentialCommandGroup(new Pause(1.5), ToGround.getToGround(s_Elevator, s_Slider, s_Wrist))),
            */

            Blue3ScoreLeftScore.followTrajectoryCommand(false, s_Swerve),
            /* for final. Purpose is to put wrist down before we get to the end in an attempt to save as much time as possible.
            new ParallelCommandGroup(Blue2ScoreLeftScore.followTrajectoryCommand(false, s_Swerve), 
            new SequentialCommandGroup(new Pause(1.5), ToHigh.getToHigh(s_Elevator, s_Slider, s_Wrist))),
            */

            //should be able to import Blue2ScoreLeft however just going to keep code above for now.

            Blue3ScoreLeftRamp.followTrajectoryCommand(false, s_Swerve)
            // for final. Purpose is to put wrist down before we get to the end in an attempt to save as much time as possible.
            //new ParallelCommandGroup(Blue2ScoreLeftScore.followTrajectoryCommand(false, s_Swerve), Stow.getStowCommand(s_Elevator, s_Slider, s_Wrist))
        );
    }
}
