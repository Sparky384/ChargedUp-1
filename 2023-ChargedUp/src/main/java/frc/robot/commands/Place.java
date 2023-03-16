package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.WristFunctionality.OuttakeAuto;
import frc.robot.commands.WristFunctionality.RotateWrist;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class Place 
{

    public static ParallelCommandGroup getPlaceCommand(Swerve swerve, Wrist wrist, Hand hand, Slider slider, Limelight s_Limelight, boolean goRight)
    {
        return new ParallelCommandGroup(
            new SequentialCommandGroup(
                //new AutoCenter(swerve, s_Limelight, goRight, Constants.LimelightConstants.LimelightCameras.LIME1),
                new RotateWrist(wrist, Constants.Subsys.wristLow),
                new OuttakeAuto(hand),
                Stow.getStowCommand(slider, wrist)
            )
        );
    }

}
