package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class contServo extends LinearOpMode {
    CRServo counterC;
    CRServo clockWise;

    @Override
    public void runOpMode() throws InterruptedException {
        counterC = hardwareMap.crservo.get("servo1");
        clockWise = hardwareMap.crservo.get("servo2");

        waitForStart();
        while (opModeIsActive()) {
            counterC.setPower(1);//This one forum I read says it might not work if the power is set too high. I'll assume that problem doesn't exist rn, but set it to .79 if it does
            clockWise.setPower(-1);

        }
    }
}
