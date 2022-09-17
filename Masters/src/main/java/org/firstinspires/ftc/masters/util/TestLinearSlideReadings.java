package org.firstinspires.ftc.masters.util;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
public class TestLinearSlideReadings extends LinearOpMode {
    DcMotor linearSlideMotor;

    @Override
    public void runOpMode() throws InterruptedException {

        linearSlideMotor = hardwareMap.dcMotor.get("");
        int linearSlideOn = 0;

        waitForStart();

        while (opModeIsActive()) {
            if(gamepad1.x && !(linearSlideOn == 1)) {
                if(linearSlideMotor.getPower() != 0) linearSlideMotor.setPower(0);
                else linearSlideMotor.setPower(.2);
                linearSlideOn = 1;
            } else if(!gamepad1.x) linearSlideOn = 0;

            if(gamepad1.y && !(linearSlideOn == -1)) {
                if(linearSlideMotor.getPower() != 0) linearSlideMotor.setPower(0);
                else linearSlideMotor.setPower(-.2);
                linearSlideOn = -1;
            } else if(!gamepad1.y) linearSlideOn = 0;

            telemetry.addData("Linear Slide Encoder Position: ", linearSlideMotor.getCurrentPosition());
        }
    }
}
