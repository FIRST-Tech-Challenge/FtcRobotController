package org.firstinspires.ftc.teamcode.Opmode.Tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class EncderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor m1 = hardwareMap.get(DcMotor.class, "fl");
        DcMotor m2= hardwareMap.get(DcMotor.class, "bl");

        waitForStart();
        while(opModeIsActive()) {
            telemetry.addData("m1", m1.getCurrentPosition());
            telemetry.addData("m2", m2.getCurrentPosition());
            telemetry.update();
        }
    }
}
