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
        public int ticks = 3850;

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
        frontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Getting some weird behaviour here where one motor is at -4500 or something
        // while the other one is at 0 or 1
        // - I believe it is because poor sensor connections by the build team
        // Poor illustration below cause i got bored waiting...
        /*
        //
        //   _ -> rightPos = 0  which is not correct
        //   |          _ -> leftPos = 0
        //   |  .____.  |
        // [[============]] <- robot
        */

        int rightStartingPos = rightDrive.getCurrentPosition();
        int leftStartingPos = leftDrive.getCurrentPosition();

        // TODO: Find the root cause of the problem (inaccurate pos readings)
        /// TEMP FIX TO COMBAT POOR WIRING ERRORS
        leftStartingPos = 0;
        rightStartingPos = 0;
        ///

        int frontStartingPos = frontDrive.getCurrentPosition();

        rightDrive.setTargetPosition(rightStartingPos);
        leftDrive.setTargetPosition(leftStartingPos);
        //frontDrive.setTargetPosition(frontStartingPos);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //frontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();
        if (opModeIsActive()) {

            // Pre-run
            while (opModeIsActive()) {
//                // Extend / Retract
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

                // TODO: Change to targetposition movement once motor is actually fixed
                /// ARM MECHANISM TEST - TEMPORARY///
                // Arad requested me to program this to use power and NOT targets
                // Dont blame me
                if(gamepad1.y) {
                    frontDrive.setPower(-PARAMS.power);
                } else if (gamepad1.a) {
                    frontDrive.setPower(PARAMS.power);
                }
                else {
                    frontDrive.setPower(0);
                }

                /* ##################################################
                             TELEMETRY ADDITIONS
               ################################################## */
                telemetry.addData("Right: ", rightDrive.getCurrentPosition());
                telemetry.addData("Left: ",  leftDrive.getCurrentPosition());
                telemetry.addData("Front: ", frontDrive.getCurrentPosition());

                telemetry.update();
            }
        }
    }
}
