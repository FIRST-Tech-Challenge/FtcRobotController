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
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");

//        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reverse right motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Drive drive = new Drive(motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight);
        //Odometry odometry = new Odometry(leftEncoder, rightEncoder, perpendicularEncoder);

        waitForStart();

        if (isStopRequested()) return;


        while (opModeIsActive()) {
            double power = gamepad1.left_stick_y/3; // Remember, this is reversed!
            double strafe = -gamepad1.left_stick_x * 1.1/3; // Counteract imperfect strafing
            double turn = -gamepad1.right_stick_x/3;
            
//            double power = gamepad2.left_stick_y/3; // Remember, this is reversed!
//            double strafe = -gamepad2.left_stick_x * 1.1/3; // Counteract imperfect strafing
//            double turn = -gamepad2.right_stick_x/3;

            drive.mecanum(power, strafe, turn);

            telemetry.addData("Back_Left Encoder: ", motorBackLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}
