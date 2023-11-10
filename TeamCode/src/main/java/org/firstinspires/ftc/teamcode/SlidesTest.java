package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class SlidesTest extends LinearOpMode {
    Bot bot = Bot.getInstance(this);
    private GamepadEx gp1;

    @Override
    public void runOpMode() throws InterruptedException {
        gp1 = new GamepadEx(gamepad1);
        slidesRunToManual(gp1.getLeftY());

        if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
            bot.slides.runTo(1);
        }else if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            bot.slides.runTo(2);
        }else if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            bot.slides.runTo(3);
        }else if(gp1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
            bot.slides.runTo(4);
        }

    }

    public void slidesRunToManual(double raw){
        bot.slides.runTo(raw*1800);
    }


}
