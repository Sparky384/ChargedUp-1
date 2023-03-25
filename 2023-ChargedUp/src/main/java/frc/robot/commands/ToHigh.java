package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.Wrist;

public class ToHigh 
{

    public static CommandBase getToHigh(Elevator elevator, Slider slider, Wrist wrist)
    {
        return new ParallelCommandGroup(
            elevator.elevatorMotionMagic(Constants.Subsys.elevatorHigh),
            new MoveSlider(slider, Constants.Subsys.sliderOut, 1.65), //delay is in seconds, 1.25
            wrist.wristMotionMagic(Constants.Subsys.wristHighGoal)
        );
    }
}
