package org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Commands.SlideCommands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.TeleOps.MainCode.TeleOps.TeleOps.MainCode.Subsystems.Slides;

public class SlidesManualCommand extends CommandBase {
    private final Slides slides;
    private final GamepadEx mechanisms;
    private final double up = 1.0;
    private final double down = -0.7;
    private final double slowUp = 0.65;
    private final double slowDown = -0.01;

    public SlidesManualCommand(Slides slides, GamepadEx mechanisms) {
        addRequirements(slides);

        this.slides = slides;
        this.mechanisms = mechanisms;
    }

    public boolean isActive() {
        return mechanisms.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON);
    }

    @Override
    public void execute() {
        boolean slow = mechanisms.getButton(GamepadKeys.Button.X);

        if (mechanisms.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON) && slides.atUpper()) {
            slides.slidePower((slow) ? slowUp : up);
        }
    }
}
