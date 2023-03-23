package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.Wrist;

public class ToGround
{

    public static CommandBase getToGround(Elevator elevator, Slider slider, Wrist wrist)
    {
        return new ParallelCommandGroup(
            elevator.elevatorMotionMagic(Constants.Subsys.elevatorLow),
            new MoveSlider(slider, Constants.Subsys.sliderIn, 0.0),
            wrist.wristMotionMagic(Constants.Subsys.wristGround)
        );
    }
}
