package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "ArmTest")
public class ArmTest extends LinearOpMode {

    public static class Params {
        public double power = 1;
        public int ticks = 3500;
        public double copen = 0.48;
        public double cclose= 0.448;
        public double slup = 0.5;

        public double lext = 0.5;
    }
    public static Params PARAMS = new Params();

    @Override
    public void runOpMode() {
        DcMotor leftDrive  = hardwareMap.get(DcMotor.class, "armLeft");
        DcMotor rightDrive = hardwareMap.get(DcMotor.class, "armRight");
        DcMotor frontDrive = hardwareMap.get(DcMotor.class, "frontArm");

        DcMotor spinner  = hardwareMap.get(DcMotor.class, "spinner");

        Servo lefts = hardwareMap.servo.get("leftSlider");
        Servo rights = hardwareMap.servo.get("rightSlider");
        Servo claw = hardwareMap.servo.get("clawServo");

        CRServo spinl = hardwareMap.crservo.get("leftSpinner");
        //Servo spinr = hardwareMap.servo.get("rightSpinner");

//        CRServo slideServo = hardwareMap.get(CRServo.class, "slideServo");

        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


//        rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        // make the motors brake when [power == 0]
        // should stop the elevator from retracting because of gravity...
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        int rightStartingPos = rightDrive.getCurrentPosition();
        int leftStartingPos = leftDrive.getCurrentPosition();


        rightDrive.setTargetPosition(rightStartingPos);
        leftDrive.setTargetPosition(leftStartingPos);
        //frontDrive.setTargetPosition(frontStartingPos);

        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinl.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();
        if (opModeIsActive()) {

            // Pre-run
            while (opModeIsActive()) {
                // Extend / Retract
                if (gamepad2.right_bumper) {
                    spinl.setPower(1);
                    //spinr.setPosition(PARAMS.lext);
                } else if (gamepad2.left_bumper) {
                    spinl.setPower(0);
                    //spinr.setPosition(0);
                }

                // TODO: Change to targetposition movement once motor is actually fixed
                /// ARM MECHANISM TEST - TEMPORARY ///
                // Arad requested me to program this to use power and NOT targets
                // Dont blame me
                if(gamepad2.y) {
                    frontDrive.setPower(-PARAMS.power);
                } else if (gamepad2.a) {
                    frontDrive.setPower(PARAMS.power);
                }
                else {
                    frontDrive.setPower(0);
                }
                if (gamepad2.x) {
                    spinner.setPower(1);
                }
                else spinner.setPower(0);

                if (gamepad2.dpad_up) {
                    lefts.setPosition(PARAMS.slup);
                    rights.setPosition(0);
                }
                else if(gamepad2.dpad_down) {
                    lefts.setPosition(0);
                    rights.setPosition(PARAMS.slup);
                }

                if (gamepad2.dpad_left) {
                    claw.setPosition(PARAMS.copen);
                } else if (gamepad2.dpad_right) {
                    claw.setPosition(PARAMS.cclose);
                }

                /* ##################################################
                             TELEMETRY ADDITIONS
                   ################################################## */
                telemetry.addData("Right: ", rightDrive.getCurrentPosition());
                telemetry.addData("Left: ",  leftDrive.getCurrentPosition());
                telemetry.addData("Front: ", frontDrive.getCurrentPosition());
                telemetry.addData(":(", spinl.getPower());

                telemetry.update();
            }
        }
    }
}
