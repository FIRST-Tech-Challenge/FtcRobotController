package org.firstinspires.ftc.teamcode.Slidy_PPV2.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Slidy_PPV2.subsystems.FrontSliderSubsystem;

import java.util.function.DoubleSupplier;

public class FrontSliderManualCommand extends CommandBase {
    private final FrontSliderSubsystem front_slider;
    private final DoubleSupplier speed;
    private final double threshold = 0.05;

    public FrontSliderManualCommand(FrontSliderSubsystem front_slider, DoubleSupplier speed){
        this.front_slider = front_slider;
        this.speed = speed;
        addRequirements(this.front_slider);
    }

    @Override
    public void execute() {
        double speed_now = -speed.getAsDouble();
        if (speed_now < -threshold || speed_now > threshold)
            front_slider.manual(speed_now/2);
        else
            front_slider.stop();
//        else if (front_slider.getState() == front_slider.State.CLOSING)
//            front_slider.close();
    }

//    @Override
//    public void end(boolean interrupted) {
//        elevator.stop();
//    }

//    @Override
//    public boolean isFinished() {
//        return false;
//    }
}
