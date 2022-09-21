package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MecanumTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // declare motors
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        DcMotor leftEncoder = hardwareMap.dcMotor.get("leftEncoder");
        DcMotor rightEncoder = hardwareMap.dcMotor.get("motorBackRight");
        DcMotor perpendicularEncoder = hardwareMap.dcMotor.get("perpendicularEncoder");

        // reverse right motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Drive drive = new Drive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);
        Odometry odometry = new Odometry(leftEncoder, rightEncoder, perpendicularEncoder);

        waitForStart();

        odometry.runOdom();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double power = -gamepad1.left_stick_y; // Remember, this is reversed!
            double strafe = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double turn = gamepad1.right_stick_x;

            drive.mecanum(power, strafe, turn);

            telemetry.addData("X: ", odometry.getX());
            telemetry.addData("Y: ", odometry.getY());
            telemetry.addData("Heading: ", odometry.getHeading());
            telemetry.update();
        }
    }
}
