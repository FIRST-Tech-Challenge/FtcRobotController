package org.firstinspires.ftc.masters;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Vslidetest")
public class verticalSlideTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor backSlides = hardwareMap.dcMotor.get("backSlides");

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                backSlides.setPower(.8);
            } else if (gamepad1.dpad_down) {
                backSlides.setPower(-.8 );
            } else {
                backSlides.setPower(0);
            }


        }
    }
}
