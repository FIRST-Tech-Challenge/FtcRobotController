package org.firstinspires.ftc.teamcode.TeleOp.UntestedFunctionsNew;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
@Disabled
public class contServo extends LinearOpMode {
    CRServo continServo;

    @Override
    public void runOpMode() throws InterruptedException {
        continServo = hardwareMap.crservo.get("continServo");

        waitForStart();
        while (opModeIsActive()) {
            continServo.setPower(1);//This one forum I read says it might not work if the power is set too high. I'll assume that problem doesn't exist rn, but set it to .79 if it does

        }
    }
}
