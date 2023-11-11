package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SlidesTest extends LinearOpMode {
    Bot bot;
    private double driveSpeed = 1;
    private GamepadEx gp1;
    private GamepadEx gp2;


    @Override
    public void runOpMode() throws InterruptedException {
        bot = Bot.getInstance(this);
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        bot.reverseMotors();


        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {

            gp1.readButtons();
            gp2.readButtons();

            telemetry.addData("TeleOp has started","wheeeee");

            drive();

            bot.slides.runToManual(gp2.getLeftY());
/*
            slidesRunToManual(gp2.getLeftY());

            if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)){
                bot.slides.runTo(1);
            }else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
                bot.slides.runTo(2);
            }else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                bot.slides.runTo(3);
            }else if(gp2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)){
                bot.slides.runTo(4);
            }


 */
            if(gp2.wasJustPressed(GamepadKeys.Button.A)){
                bot.fourbar.outtakeTest();
                telemetry.addData("fourbar should be in outtake position",bot.fourbar.getOuttakePos());
            }
            else if(gp2.wasJustPressed(GamepadKeys.Button.B)){
                bot.fourbar.storageTest();
                telemetry.addData("fourbar should be in storage pos",bot.fourbar.getStoragePos());
            }


            telemetry.update();
        }


    }

    public void slidesRunToManual(double raw){
        bot.slides.runTo(raw*1800);
    }

    private void drive() {
        gp1.readButtons();
        driveSpeed = 1;
        driveSpeed *= 1 - 0.5 * gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        driveSpeed = Math.max(0, driveSpeed);
        Vector2d driveVector = new Vector2d(gp1.getLeftX(), -gp1.getLeftY()),
                turnVector = new Vector2d(
                        gp1.getRightX(), 0);
        bot.driveRobotCentric(-driveVector.getX() * driveSpeed,
                -driveVector.getY() * driveSpeed,
                turnVector.getX() * driveSpeed / 1.7
        );
    }

}
