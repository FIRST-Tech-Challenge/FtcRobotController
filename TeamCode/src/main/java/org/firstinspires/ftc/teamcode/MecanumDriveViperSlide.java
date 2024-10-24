package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Viper Slide")
public class MecanumDriveViperSlide extends OpMode {

    DcMotor frontLeftMotor = null;
    DcMotor backLeftMotor = null;
    DcMotor frontRightMotor = null;
    DcMotor backRightMotor = null;
    public DcMotor viperSlideMotor = null;
    public DcMotor armMotor = null;

    final double ARM_POWER = 0.5d;
    final double VIPER_SLIDE_POWER = 0.5d;

    @Override
    public void init() {
         frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
         backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
         frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
         backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
         viperSlideMotor = hardwareMap.get(DcMotor.class, "viperSlideMotor");
         armMotor = hardwareMap.get(DcMotor.class, "left_arm");

         armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {

        if(gamepad1.dpad_up)
            armMotor.setPower(ARM_POWER);
        else if (gamepad1.dpad_down)
            armMotor.setPower(ARM_POWER * -1);
        else
            armMotor.setPower(0);

        if(gamepad1.x)
            viperSlideMotor.setPower(VIPER_SLIDE_POWER);
        else if (gamepad1.y)
            viperSlideMotor.setPower(VIPER_SLIDE_POWER * -1);
        else
            viperSlideMotor.setPower(0);

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

}
