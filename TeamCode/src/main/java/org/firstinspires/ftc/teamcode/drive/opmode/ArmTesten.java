package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ArmTesten extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        DcMotor armMotor1 = hardwareMap.dcMotor.get("arm1");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double armPower = -gamepad2.left_stick_y;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]

            if (armMotor1.getCurrentPosition() < 870) {
                armMotor1.setPower(armPower + 0.3);
            }
            else if (armPower < 0  && armMotor1.getCurrentPosition() < 0) {
                armMotor1.setPower(0);
            }
            else {
                armMotor1.setPower(0.1);
            }

//            runTo(intakeMotor, intakeMotorPos);

            telemetry.addData("armPosition1", armMotor1.getCurrentPosition());
            telemetry.update();
        }
    }
}