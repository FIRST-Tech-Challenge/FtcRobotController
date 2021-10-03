package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class TestDrive extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor rearLeft;
    DcMotor rearRight;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("TopLeft");
        frontRight = hardwareMap.dcMotor.get("TopRight");
        rearLeft = hardwareMap.dcMotor.get("BackLeft");
        rearRight = hardwareMap.dcMotor.get("BackRight");


        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);


    }

    @Override
    public void loop() {
        frontLeft.setPower(-gamepad1.left_stick_y);
        rearLeft.setPower(-gamepad1.left_stick_y);

        frontRight.setPower(-gamepad1.right_stick_y);
        rearRight.setPower(-gamepad1.right_stick_y);
    }
}
