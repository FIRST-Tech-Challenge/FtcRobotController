package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

//add any test stuff you need to do here
@TeleOp
public class testTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor intake = hardwareMap.dcMotor.get("intake");
        DcMotor flippyThingy = hardwareMap.dcMotor.get("flipper");
        flippyThingy.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flippyThingy.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a == true) {
                intake.setPower(0.6);

            } else if (gamepad1.b == true) {
                intake.setPower(-0.6  );
            } else {
                intake.setPower(0);
            }

            if (gamepad1.x == true) {
                flippyThingy.setPower(0.6);

            } else if (gamepad1.y == true) {
                flippyThingy.setPower(-0.6);
            } else {
                flippyThingy.setPower(0);
            }
        }
    }
}
