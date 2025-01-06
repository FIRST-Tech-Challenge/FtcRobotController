package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "ArmTest")
public class ArmTest extends LinearOpMode {

    public static class Params {
        public double power = 1;
        public int ticks = 3930;

    }
    public static Params PARAMS = new Params();

    @Override
    public void runOpMode() {
        DcMotor leftDrive  = hardwareMap.get(DcMotor.class, "armLeft");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "armRight");
        DcMotor frontDrive = hardwareMap.get(DcMotor.class, "frontArm");
        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        // make the motors brake when [power == 0]
        // should stop the elevator from retracting because of gravity...
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int rightStartingPos = rightDrive.getCurrentPosition();
        int leftStartingPos = leftDrive.getCurrentPosition();

        rightDrive.setTargetPosition(rightStartingPos);
        leftDrive.setTargetPosition(leftStartingPos);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        if (opModeIsActive()) {

            // Pre-run
            while (opModeIsActive()) {
                // Extend / Retract
                if (gamepad1.right_bumper) {
                    rightDrive.setPower(PARAMS.power);
                    leftDrive.setPower(PARAMS.power);
                    rightDrive.setTargetPosition(PARAMS.ticks);
                    leftDrive.setTargetPosition(PARAMS.ticks);
                } else if (gamepad1.left_bumper) {
                    rightDrive.setPower(PARAMS.power);
                    leftDrive.setPower(PARAMS.power);
                    rightDrive.setTargetPosition(rightStartingPos);
                    leftDrive.setTargetPosition(leftStartingPos);
                }

                /// ARM MECHANISM TEST ///
                if(gamepad1.y) {
                    frontDrive.setPower(0.1);
                } else if (gamepad1.a) {
                    frontDrive.setPower(-0.1 );
                }

                /// Telemetry
                telemetry.addData("Right: ", rightDrive.getCurrentPosition());
                telemetry.addData("Left: ",  leftDrive.getCurrentPosition());
                telemetry.addData("Front: ", frontDrive.getCurrentPosition());

                telemetry.update();
            }
        }
    }
}
