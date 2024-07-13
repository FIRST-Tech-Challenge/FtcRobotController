package org.firstinspires.ftc.teamcode.opmode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Dropper;

@Config
@TeleOp(name = "DropperTest", group = "Test")

public class DropperTest extends LinearOpMode {

    private Dropper dropper;

    static public boolean loggingOn = true;
    private boolean loading = false;
    @Override
    public void runOpMode() {

        dropper = new Dropper(hardwareMap, telemetry, loggingOn);
        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_bumper) {
                dropper.load();
                loading = true;
            } else if (gamepad1.left_bumper) {
                dropper.stopLoad();
                loading = false;
            }
            if (gamepad1.square) {
                dropper.dropPixel();
                sleep(200);
            }

            dropper.update();
            if (dropper.fullyLoaded() && loading)
            {
                loading = false;
                dropper.stopLoad();
            }
        }
    }
}
