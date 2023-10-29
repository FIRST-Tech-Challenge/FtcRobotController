package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "FINALCODE")
public class PullUp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor pullUpMotor = hardwareMap.dcMotor.get("motor1");
        pullUpMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        while (opModeIsActive()) {
            if (gamepad1.x) {
                pullUpMotor.setPower(1);
            }
            else if (gamepad1.y) {
                pullUpMotor.setPower(-1);
            }
            else {
                pullUpMotor.setPower(0);
            }
        }
    }
}