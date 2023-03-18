package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Slider;

public class ToMid
{

    public static CommandBase getToMid(Elevator elevator, Slider slider)
    {
        return new SequentialCommandGroup(
            elevator.elevatorMotionMagic(Constants.Subsys.elevatorLow),
            new MoveSlider(slider, Constants.Subsys.sliderIn),
            elevator.wristMotionMagic(Constants.Subsys.wristMid)
        );
    }
}
