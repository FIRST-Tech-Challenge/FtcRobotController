package org.firstinspires.ftc.teamcode.TeleOps.MainCode.Commands.SlideCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeleOps.MainCode.Subsystems.Slides;

public class SlidesResetCommand extends CommandBase {
    private final Slides slides;
    private final GamepadEx mechanisms;

    public SlidesResetCommand(Slides slides, GamepadEx mechanisms) {
        addRequirements(slides);

        this.slides = slides;
        this.mechanisms = mechanisms;
    }

    @Override
    public void execute() {
        if (mechanisms.getButton(GamepadKeys.Button.DPAD_DOWN)){
            slides.slidePower(-0.2);
        } else {
            slides.end();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted){
        slides.end();
        slides.resetSlidePos();
    }
}
