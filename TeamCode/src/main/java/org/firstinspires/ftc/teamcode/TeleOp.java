package org.firstinspires.ftc.teamcode;

//package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Test Code")
public class TeleOp extends OpMode {
    protected DcMotor frontLeft;
    protected DcMotor frontRight;
    //protected DcMotor backLeft;
    //protected DcMotor backRight;
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "LeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "RightMotor");
        //backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        //backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        //backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        if (gamepad1.left_stick_y > 0){
            frontLeft.setPower(1);
            frontRight.setPower(1);
            //backLeft.setPower(1);
            //backRight.setPower(1);
        }
        if (gamepad1.left_stick_y < 0){
            frontLeft.setPower(-1);
            frontRight.setPower(-1);
            //backLeft.setPower(-1);
            //backRight.setPower(-1);
        } else {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            //backLeft.setPower(0);
            //backRight.setPower(0);
        }

        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double z = gamepad1.right_stick_x;

        double multiplier = 1;
        if (gamepad1.left_trigger > 0.1) {
            multiplier *= 0.55;
        }
        if (gamepad1.right_trigger > 0.1) {
            multiplier *= 0.55;
        }
        double v1 = Range.clip(y - x + z, -multiplier, multiplier);
        double v2 = Range.clip(y + x - z, -multiplier, multiplier);
        //double v3 = Range.clip(y + x + z, -multiplier, multiplier);
        //double v4 = Range.clip(y - x - z, -multiplier, multiplier);

        frontLeft.setPower(v1);
        frontRight.setPower(v2);
        //backLeft.setPower(v3);
        //backRight.setPower(v4);
    }
}
