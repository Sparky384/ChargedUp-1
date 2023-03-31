package frc.robot.autonomous;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.paths.DistrictTest;
import frc.robot.subsystems.*;


public class DistrictTestAuto extends SequentialCommandGroup{
    private static Swerve s_Swerve;

        // Assuming this method is part of a drivetrain subsystem that provides the necessary methods
    public static CommandBase followTrajectoryCommand(Swerve swerve, Elevator elevator, Slider slider, Wrist wrist, Hand hand) {
        s_Swerve = swerve;

        return new SequentialCommandGroup(
            DistrictTest.followTrajectoryCommand(true, s_Swerve)
        );
    }
    
}
