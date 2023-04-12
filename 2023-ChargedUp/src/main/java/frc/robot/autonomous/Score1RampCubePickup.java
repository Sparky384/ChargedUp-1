package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.subsystems.Swerve;
import frc.robot.commands.WristFunctionality.*;
import frc.robot.commands.DriveFunctionality.*;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.Constants;
import frc.robot.commands.Pause;
import frc.robot.commands.CommandGroups.ToGroundAutoCube;
import frc.robot.subsystems.*;

public class Score1RampCubePickup extends SequentialCommandGroup {
    private static Swerve s_Swerve;
    private static Hand s_Hand;

    

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;
        s_Hand = hand;

        return new SequentialCommandGroup(
            new IntakeAuto(s_Hand),
            new Drive(s_Swerve, -0.40, 0, 3),
            new Pause(1.25), //may be able to take out I don't know yet.
            new ParallelCommandGroup(
                ToGroundAutoCube.getToGround(elevator, slider, wrist), 
                new TurnTo(s_Swerve, 30, 1)),
            new ParallelRaceGroup( //using race group because Outtake can't normally end.
                new Outtake(hand), 
                new Drive(s_Swerve, -0.20, 0, 0.5)), //time is not final.
            new Drive(s_Swerve, 0.20, 0, 0.5), //time is not final.
            new TurnTo(s_Swerve, 180, 1), //0 would be quicker if necessary, however, 180 makes it so they face the scoring area first to quickly score.
            new DriveOnRamp(s_Swerve, true),
            new GyroStabalize(s_Swerve)
            
            /* 
            //could be used to shoot a cube from the top of the ramp.
            new ParallelCommandGroup(
                wrist.wristMotionMagic(Constants.Subsys.wristHigh), 
                new MoveSlider(slider, Constants.Subsys.sliderIn, 0.0), 
                elevator.elevatorMotionMagic(Constants.Subsys.elevatorHigh)
            ),
            new IntakeAuto(s_Hand)
            */
        );
    }
}
