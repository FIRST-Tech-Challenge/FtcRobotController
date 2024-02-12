package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "mainOpModeMecanum", group = "official")
public class mainOpModeMecanum extends LinearOpMode {

    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private Servo plane;
    private DcMotor rollIn;
    private DcMotor dualArm;
    private Servo garbageCollector;

    int flag2;
    int flag1;
    int prevValue;

    /**
     * Describe this function...
     */
    private void run() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        
        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        plane = hardwareMap.get(Servo.class, "plane");
        rollIn = hardwareMap.get(DcMotor.class, "rollIn");
        dualArm = hardwareMap.get(DcMotor.class, "dualArm");
        garbageCollector = hardwareMap.get(Servo.class, "garbageCollector");

        // Put initialization blocks here.
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            flag1 = 0;
            prevValue = 0;
            while (opModeIsActive()) {
                run();
                roll();
                armUp_Down();
                collectGarbage();
                checkAlternation();
                if (flag1 / 2 >= 5) {
                    releasePlane();
                } else {
                    plane.setPosition(0);
                }
                telemetry.addData("flag1", flag1);
                telemetry.addData("flag2", flag2);
                telemetry.update();
            }
        }
    }

    /**
     * Describe this function...
     */
    private void roll() {
        rollIn.setPower(0);
        if (gamepad1.left_bumper) {
            rollIn.setDirection(DcMotorSimple.Direction.FORWARD);
            rollIn.setPower(1);
        }
        if (gamepad1.right_bumper) {
            rollIn.setDirection(DcMotorSimple.Direction.REVERSE);
            rollIn.setPower(1);
        }
    }

    /**
     * Describe this function...
     */
    private void armUp_Down() {
        dualArm.setPower(0);
        if (gamepad1.a) {
            dualArm.setDirection(DcMotorSimple.Direction.REVERSE);
            dualArm.setPower(0.3);
        }
        if (gamepad1.y) {
            dualArm.setDirection(DcMotorSimple.Direction.FORWARD);
            dualArm.setPower(0.3);
        }
    }

    /**
     * Describe this function...
     */
    private void releasePlane() {
        if (gamepad1.b) {
            plane.setPosition(1);
        } else {
            plane.setPosition(0);
        }
    }

    /**
     * Describe this function...
     */
    private void collectGarbage() {
        if (gamepad1.left_trigger > 0) {
            garbageCollector.setPosition(0);
        }
        if (gamepad1.right_trigger > 0) {
            garbageCollector.setPosition(1);
        }
    }

    /**
     * Describe this function...
     */
    private void checkAlternation() {
        if (gamepad1.b) {
            flag2 = 1;
        } else {
            flag2 = 0;
        }
        if (flag2 != prevValue) {
            flag1 += 1;
            prevValue = flag2;
        }
    }
}