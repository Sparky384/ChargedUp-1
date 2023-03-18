package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Slider;

public class ToFeeder 
{

    public static CommandBase getToFeeder(Elevator elevator, Slider slider)
    {
        return new SequentialCommandGroup(
            elevator.elevatorMotionMagic(Constants.Subsys.elevatorHigh),
            new MoveSlider(slider, Constants.Subsys.sliderOut),
            elevator.wristMotionMagic(Constants.Subsys.wristMid)
        );
    }
}
