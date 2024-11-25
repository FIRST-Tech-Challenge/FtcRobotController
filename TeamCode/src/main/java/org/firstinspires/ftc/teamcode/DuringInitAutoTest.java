package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class DuringInitAutoTest extends LinearOpMode {
    //RobotMain bart;

    GamepadEx gamepadEx;

    @Override
    public void runOpMode() throws InterruptedException {
        //bart = new RobotMain(hardwareMap, telemetry);
        gamepadEx = new GamepadEx(gamepad1);

        double value = 0;
        boolean upIsPositive = false;
        while (!isStarted()) {
            gamepadEx.readButtons();

            if (upIsPositive) {
                value = RobotMain.dpadInputToChangeValueUpIsPositive(value, gamepadEx);
            } else {
                value = RobotMain.dpadInputToChangeValueUpIsNegative(value, gamepadEx);
            }

            if (gamepadEx.wasJustPressed(GamepadKeys.Button.A)) {
                upIsPositive = !upIsPositive;
            }

            telemetry.addData("value", value);
            telemetry.addData("upIsPositive", upIsPositive);
            telemetry.update();
        }

        waitForStart();
    }


}
