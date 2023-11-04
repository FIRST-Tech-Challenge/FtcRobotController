package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(group="ZTest")
public class encoderTest extends LinearOpMode {
    public void runOpMode() throws InterruptedException{
        DcMotor testEncoder = hardwareMap.dcMotor.get("motor4");
        waitForStart();
        if (isStopRequested()) return;
        while (opModeIsActive()) {
         telemetry.addData("odometer pos", testEncoder.getCurrentPosition());
         telemetry.update();
        }
    }
}
