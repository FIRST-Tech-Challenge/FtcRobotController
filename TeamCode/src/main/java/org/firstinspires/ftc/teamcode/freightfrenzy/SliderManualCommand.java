package org.firstinspires.ftc.teamcode.freightfrenzy;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

public class SliderManualCommand extends CommandBase {
    private final SliderSubsystem slider;
    private final int direction;
    private static final int step = 20;

    public SliderManualCommand(SliderSubsystem slider, int direction){
        this.slider = slider;
        this.direction = direction;
        addRequirements(this.slider);
    }

    @Override
    public void initialize() {
//        slider.setHeight(slider.getHeight() + step*direction);
        slider.setManual();
    }

    @Override
    public void execute() {
        slider.setPower(0.4*direction);
    }

    @Override
    public void end(boolean interrupted) {
        slider.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }


}
