package org.firstinspires.ftc.teamcode.readyTestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Lift Test Code")
public class LiftTestCode extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor lift = hardwareMap.dcMotor.get("lift");
        lift.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.y) {
                lift.setPower(1);
            }else if (gamepad1.a) {
                lift.setPower(-1);
            }else {
                lift.setPower(0);
            }


        }
    }
}