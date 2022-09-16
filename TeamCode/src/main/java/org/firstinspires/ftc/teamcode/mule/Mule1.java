package org.firstinspires.ftc.teamcode.mule;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Mule1")
public class Mule1 extends OpMode {

    DcMotor FR = null;
    DcMotor FL = null;
    DcMotor BR = null;
    DcMotor BL = null;
    DcMotor Arm = null;

    double drive;
    double strafe;
    double rotate;

    double FLPower;
    double FRPower;
    double BLPower;
    double BRPower;

    double ArmPower =0.5;

    @Override
    public void init() {
        //Connect Motor
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");

        Arm = hardwareMap.dcMotor.get("Arm");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //Set Up Motor Direction
        FR.setDirection(DcMotor.Direction.FORWARD);
        FL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("STATUS", "Initialized");

    }

    @Override
    public void loop() {
        drive = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        FLPower = (drive + strafe + rotate);
        FRPower = (drive - strafe - rotate);
        BLPower = (drive - strafe + rotate);
        BRPower = (drive + strafe - rotate);

        FL.setPower(-FLPower);
        FR.setPower(-FRPower);
        BL.setPower(-BLPower);
        BR.setPower(-BRPower);

        if(gamepad1.dpad_up) {
            Arm.setPower(ArmPower);

        }
        else {
            Arm.setPower(0);
        }
        if (gamepad1.dpad_down) {
            Arm.setPower(-ArmPower);
        }
        else {
            Arm.setPower(0);
        }
    }
}
