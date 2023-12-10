package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Hardware;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "drive")
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Hardware hardware = new Hardware(hardwareMap);
        waitForStart();
//        ElapsedTime updateDelta = new ElapsedTime();
        while (opModeIsActive()) {
            double y = gamepad1.right_stick_x;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.left_stick_y;
            double ly2 = gamepad2.left_stick_y;
            double ry2 = gamepad2.right_stick_y;
            boolean a = gamepad2.a; // extend
            boolean b = gamepad2.b; // retract

            hardware.frontLeft.setPower(y + x + rx);
            hardware.frontRight.setPower(y - x - rx);
            hardware.backLeft.setPower(y - x + rx);
            hardware.backRight.setPower(y + x - rx);

//            SlidingArmVD arm1 = new SlidingArmVD("part", "arm1", new HashMap<String, Integer>(), 0, hardware.armMotor1);
//            SlidingArmVD arm2 = new SlidingArmVD("part", "arm1", new HashMap<String, Integer>(), 0, hardware.armMotor2);
            hardware.armMotor1.setPower(ly2);
            hardware.armMotor2.setPower(ry2);
            if (a) {
                hardware.droneServo.setDirection(Servo.Direction.REVERSE);
                hardware.droneServo.setPosition(0.5);
            }
//            arm1.runWithController(ly2, updateDelta);
//            arm2.runWithController(ry2, updateDelta);
//            updateDelta.reset();
        }
    }
}
