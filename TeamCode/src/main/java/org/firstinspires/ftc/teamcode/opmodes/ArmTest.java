package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ArmTest")
public class ArmTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        DcMotor leftDrive  = hardwareMap.get(DcMotor.class, "armLeft"); // port 0
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "armRight"); // port 3
        waitForStart();
        if (opModeIsActive()) {

            // Pre-run
            while (opModeIsActive()) {
                //////////////////////////////////////////////////////////////////////////////////////
                /// TEST FOR ELEVATOR OPERATION - NOT TESTED DUE TO THE ROBOT BEING A LITTLE BITCH ///
                //////////////////////////////////////////////////////////////////////////////////////

                // make the motors brake when [power == 0]
                // should stop the elevator from retracting because of gravity...
                rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

                // Extend / Retract
                if (gamepad1.right_bumper) {
                    rightDrive.setPower(0.1);
                    leftDrive.setPower(0.1);
                } else if (gamepad1.left_bumper) {
                    rightDrive.setPower(-0.1);
                    leftDrive.setPower(-0.1);
                }
                // Brake
                else {
                    rightDrive.setPower(0);
                    leftDrive.setPower(0);
                }

                /// Telemetry
                telemetry.addData("Right: ", rightDrive.getPower());
                telemetry.addData("Left: ",  leftDrive.getPower());
            }
        }
    }
}
