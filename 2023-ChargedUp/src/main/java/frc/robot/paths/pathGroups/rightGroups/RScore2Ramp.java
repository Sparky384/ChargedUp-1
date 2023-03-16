package frc.robot.paths.pathGroups.rightGroups;

import frc.lib.pathplanner.com.pathplanner.lib.PathPlanner;
import frc.lib.pathplanner.com.pathplanner.lib.PathPlannerTrajectory;
import frc.lib.pathplanner.com.pathplanner.lib.commands.PPSwerveControllerCommand;

import javax.lang.model.element.Element;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.subsystems.*;
import frc.lib.pathplanner.com.pathplanner.lib.PathConstraints;
import frc.robot.Constants;
import frc.robot.paths.rightPaths.RRampFrom2nd;
import frc.robot.commands.DriveOnRamp;
import frc.robot.commands.GyroStabalize;
import frc.robot.commands.ElevatorFunctionality.MoveElevator;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.*;


public class RScore2Ramp extends SequentialCommandGroup {
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
            /* Call RScore2: Elevator is still up after scoring, sliderIn, WristHigh */
            RScore2.followTrajectoryCommand(s_Swerve, s_Elevator, s_Slider, s_Wrist, s_Hand),
            
            /* Drop elevator while moving to ramp. */
            new ParallelCommandGroup(new MoveElevator(s_Elevator, Constants.Subsys.elevatorLow), RRampFrom2nd.followTrajectoryCommand(false, s_Swerve)),
            
            /* Drive onto ramp then balance indefinitely */
            new DriveOnRamp(s_Swerve, true),
            new GyroStabalize(s_Swerve)
        );
    }
}
