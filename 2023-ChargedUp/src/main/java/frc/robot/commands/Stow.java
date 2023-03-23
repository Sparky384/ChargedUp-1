package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.commands.WristFunctionality.RotateWrist;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.Wrist;

public class Stow 
{

    public static ParallelCommandGroup getStowCommand(Elevator elevator, Slider slider, Wrist wrist)
    {
        return new ParallelCommandGroup(
            wrist.wristMotionMagic(Constants.Subsys.wristHigh),
            new MoveSlider(slider, Constants.Subsys.sliderIn, 0.0),
            elevator.elevatorMotionMagic(Constants.Subsys.elevatorLow)
        );
    }
    
}
