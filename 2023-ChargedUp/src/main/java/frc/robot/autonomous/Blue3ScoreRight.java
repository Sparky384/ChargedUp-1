package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;

import frc.robot.paths.rightPaths.Blue3ScoreRightPickup;
import frc.robot.paths.rightPaths.Blue3ScoreRightScore;

public class Blue3ScoreRight extends SequentialCommandGroup{
    private static Swerve s_Swerve;

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;

        return new SequentialCommandGroup(
            //new IntakeAuto(s_Hand), //assumes we start with cube so this is outtake for cube
            
            Blue3ScoreRightPickup.followTrajectoryCommand(true, s_Swerve), //goes to pickup cone.
            
            /* for final. Purpose is to put wrist down before we get to the end in an attempt to save as much time as possible.
            new ParallelCommandGroup(Blue3ScoreRightPickup.followTrajectoryCommand(true, s_Swerve), 
            new SequentialCommandGroup(new Pause(1.5), ToGround.getToGround(s_Elevator, s_Slider, s_Wrist))),
            */

            Blue3ScoreRightScore.followTrajectoryCommand(false, s_Swerve)
            /* for final. Purpose is to put wrist down before we get to the end in an attempt to save as much time as possible.
            new ParallelCommandGroup(Blue3ScoreRightScore.followTrajectoryCommand(false, s_Swerve), 
            new SequentialCommandGroup(new Pause(1.5), ToHigh.getToHigh(s_Elevator, s_Slider, s_Wrist))),
            */
        );
    }
}
