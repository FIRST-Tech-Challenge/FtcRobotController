package org.firstinspires.ftc.teamcode.powerplayV2;

import com.arcrobotics.ftclib.command.CommandBase;

public class SliderCommand extends CommandBase {
    private final SliderSubsystem slider;
    private final SliderSubsystem.Level targetLevel;

    public SliderCommand(SliderSubsystem slider, SliderSubsystem.Level levelPicked){
        this.slider = slider;
        targetLevel = levelPicked;
        addRequirements(this.slider);
    }

    @Override
    public void initialize() {
        slider.setAuto();
        slider.setLevel(targetLevel);
    }

    @Override
    public void execute() {
        slider.run();
    }

    @Override
    public void end(boolean interrupted) {
        slider.stop();
    }

    @Override
    public boolean isFinished() {
        return slider.atTargetLevel();
    }


}
