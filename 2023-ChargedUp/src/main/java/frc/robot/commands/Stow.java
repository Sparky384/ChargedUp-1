package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.RotateWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.Wrist;

public class Stow 
{

    public static ParallelCommandGroup getStowCommand(Slider slider, Elevator elevator)
    {
        return new ParallelCommandGroup(
            new MoveSlider(slider, Constants.Subsys.sliderIn),
            elevator.wristMotionMagic(Constants.Subsys.wristHigh),
            elevator.elevatorMotionMagic(Constants.Subsys.elevatorLow)
        );
    }
    
}
