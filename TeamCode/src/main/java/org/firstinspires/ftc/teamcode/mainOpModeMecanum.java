package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testDrive (Blocks to Java)")
public class mainOpModeMecanum extends LinearOpMode {

    private DcMotor leftDrive;
    private DcMotor rightDrive;
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
        int forward;
        int backward;
        int left;
        int right;

        forward = 0;
        backward = 0;
        left = 0;
        right = 0;
        if (gamepad1.left_stick_y < 0) {
            forward = 1;
            backward = 0;
        } else if (gamepad1.left_stick_y > 0) {
            forward = 0;
            backward = 1;
        } else {
            forward = 0;
            backward = 0;
        }
        if (gamepad1.right_stick_x > 0) {
            right = 1;
            left = 0;
        } else if (gamepad1.right_stick_x < 0) {
            right = 0;
            left = 1;
        } else {
            right = 0;
            left = 0;
        }
        leftDrive.setPower(forward * 0.7 - (backward * 0.7 - (right * 0.3 - left * 0.3)));
        rightDrive.setPower(forward * 0.7 - (backward * 0.7 - (left * 0.3 - right * 0.3)));
    }

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        plane = hardwareMap.get(Servo.class, "plane");
        rollIn = hardwareMap.get(DcMotor.class, "rollIn");
        dualArm = hardwareMap.get(DcMotor.class, "dualArm");
        garbageCollector = hardwareMap.get(Servo.class, "garbageCollector");

        // Put initialization blocks here.
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            flag1 = 0;
            prevValue = 0;
            rightDrive.setDirection(DcMotorSimple.Direction.REVERSE);
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