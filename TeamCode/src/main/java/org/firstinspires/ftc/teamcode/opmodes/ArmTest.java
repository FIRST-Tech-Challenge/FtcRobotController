package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
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

        DcMotor spinner  = hardwareMap.get(DcMotor.class, "spinner");

//        CRServo slideServo = hardwareMap.get(CRServo.class, "slideServo");

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // make the motors brake when [power == 0]
        // should stop the elevator from retracting because of gravity...
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        int rightStartingPos = rightDrive.getCurrentPosition();
        int leftStartingPos = leftDrive.getCurrentPosition();

        int frontStartingPos = frontDrive.getCurrentPosition();

        rightDrive.setTargetPosition(rightStartingPos);
        leftDrive.setTargetPosition(leftStartingPos);
        //frontDrive.setTargetPosition(frontStartingPos);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
                    rightDrive.setTargetPosition(rightStartingPos+50);
                    leftDrive.setTargetPosition(leftStartingPos+50);
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
                if (gamepad1.x) {
                    spinner.setPower(-1.0);
                }
                else spinner.setPower(0);



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
