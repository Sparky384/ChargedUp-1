package frc.robot.commands.CommandGroups;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.SliderFunctionality.MoveSlider;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Slider;
import frc.robot.subsystems.Wrist;

public class ToFeeder 
{

    public static CommandBase getToFeeder(Elevator elevator, Slider slider, Wrist wrist)
    {
        return new ParallelCommandGroup(
            elevator.elevatorMotionMagic(Constants.Subsys.elevatorFeederStation),
            new MoveSlider(slider, Constants.Subsys.sliderIn, 0.0),
            wrist.wristMotionMagic(Constants.Subsys.wristFeederStation)
        );
    }
}
