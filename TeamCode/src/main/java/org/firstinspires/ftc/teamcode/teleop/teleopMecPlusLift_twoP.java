package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp(name = "Tele-Op_PlusLiftTwoP")
public class teleopMecPlusLift_twoP extends LinearOpMode {

    //comment to help with commit2

    @Override
    public void runOpMode() throws InterruptedException {

//        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
//        DcMotor backLeft = hardwareMap.dcMotor.get("bl");
//Based on testing, the front and back left motors were reversed.  Reversing them here (FIXED THE ISSUE!!!)

        DcMotor frontLeft = hardwareMap.dcMotor.get("fl");
        DcMotor backLeft = hardwareMap.dcMotor.get("bl");

        DcMotor frontRight = hardwareMap.dcMotor.get("fr");
        DcMotor backRight = hardwareMap.dcMotor.get("br");

        DcMotor intakeMotor = hardwareMap.dcMotor.get("intake_motor");
        DcMotor lifterMotor = hardwareMap.dcMotor.get("lifter");

        Servo intakeServo = hardwareMap.servo.get("claw");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.setAutoClear(true);
        telemetry.update();
        float driveSpeed = 0.5f;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

//            float intakeMotorPower = gamepad2.left_trigger;
//            float liftDeliverMotorPower = gamepad2.right_trigger;
            boolean intakeMotorButtom = gamepad2.b;
            boolean intakeMotorButtomReverse = gamepad2.x;

            float intakeMotorPower = 1f;

            float lifterRaiseMotorPower = gamepad2.right_trigger;
            float lifterLowerMotorPower = gamepad2.left_trigger;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower * driveSpeed);
            backLeft.setPower(backLeftPower * driveSpeed);
            frontRight.setPower(frontRightPower * driveSpeed);
            backRight.setPower(backRightPower * driveSpeed);

            //*******************
            //* Extra functions *
            //*******************
            //lifterRaise
            if (gamepad2.left_trigger > 0) {
                lifterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                lifterMotor.setPower(0.3 * (lifterLowerMotorPower));
            //lifterLower
            } else if (gamepad2.right_trigger > 0) {
                lifterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                lifterMotor.setPower(0.3 * (lifterRaiseMotorPower));
            //intake
            } else {
                lifterMotor.setPower(0);
            }

            telemetry.addData("Claw position: ", intakeServo.getPosition());

            //This is the logic that raises and lowers the claw (game position and hook position)
            if(gamepad1.dpad_up) {
                intakeServo.setPosition(0.25);
            } else if (gamepad1.dpad_down) {
                intakeServo.setPosition(0.75);
            }

            //0.25 just past straight
            //0.05

            if (intakeMotorButtom) {
                intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeMotor.setPower((intakeMotorPower));
                //no extra functions occuring
            } else if (intakeMotorButtomReverse) {
                intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeMotor.setPower((intakeMotorPower));
                //no extra functions occuring
            } else {
                intakeMotor.setPower(0);
            }
            telemetry.update();
        }
    }
}

