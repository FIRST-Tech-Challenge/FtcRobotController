package org.firstinspires.ftc.teamcode.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Testing.TeachingClass;

@TeleOp(name = "CostaNewTest", group = "TeleOp")

public class CostaNewTest extends LinearOpMode {


    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DcMotor intakeMotor;
    private DcMotor clawExtender;
    private Servo clawServo;
    private Servo droneLauncher;
    private boolean clawOpen = false;

    public void runOpMode() {
        // Initialize motors and servos
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        clawExtender = hardwareMap.get(DcMotor.class, "claw_extend_motor");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        droneLauncher = hardwareMap.get(Servo.class, "drone_launcher");


        waitForStart();

        double xSpeed = -gamepad1.left_stick_x;
        double ySpeed = gamepad1.left_stick_y;
        double rotation = gamepad1.right_stick_x;

        frontLeft.setPower(xSpeed + ySpeed + rotation);
        frontRight.setPower(-xSpeed + ySpeed - rotation);
        backLeft.setPower(xSpeed + ySpeed - rotation);
        backRight.setPower(-xSpeed + ySpeed + rotation);

        double intakePower = 0.0;
        if (gamepad2.right_bumper) {
            intakePower = 1.0;
        }
        intakeMotor.setPower(intakePower);

        double clawExtensionPower = 0.0;
        if (gamepad2.left_bumper) {
            clawExtensionPower = 1.0;
        }
        clawExtender.setPower(clawExtensionPower);

        if (gamepad2.a) {
            clawServo.setPosition(0.9);
            clawOpen = true;
        }

        if (gamepad2.b) {
            clawServo.setPosition(0.0);
            clawOpen = false;
        }

        if (gamepad2.x) {
            droneLauncher.setPosition(1.0);
        } else {
            droneLauncher.setPosition(0.0);
        }
    }

}
