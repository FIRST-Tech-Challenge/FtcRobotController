package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Hslidetest")
public class horizontalSlideTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor gpSlideLeft = hardwareMap.dcMotor.get("gpSlideLeft");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                gpSlideLeft.setPower(.8);
            } else if (gamepad1.dpad_down) {
                gpSlideLeft.setPower(-.8 );
            } else {
                gpSlideLeft.setPower(0);
            }


        }
    }
}
