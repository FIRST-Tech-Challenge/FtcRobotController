package org.firstinspires.ftc.teamcode.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp (name = "HDriveTeleOp")
public class HDriveTeleOp extends OpMode {
    DcMotorEx FrontLeft, FrontRight, BackLeft, BackRight, Middle;

    @Override
    public void init(){
        FrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("FL");
        FrontRight = (DcMotorEx) hardwareMap.dcMotor.get("FR");
        BackLeft = (DcMotorEx) hardwareMap.dcMotor.get("BL");
        BackRight = (DcMotorEx) hardwareMap.dcMotor.get("BR");
        Middle = (DcMotorEx) hardwareMap.dcMotor.get("Middle");

        FrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Middle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop(){
        double rpow;
        double lpow;
        double mpow;

        double xleft = gamepad1.left_stick_x;
        double yleft = -gamepad1.left_stick_y;
        double xright = gamepad1.right_stick_x;

        lpow = rpow = yleft;
        mpow = xright;

        if(xleft>0)
            rpow -= xleft;
        else if(xleft<0)
            lpow += xleft;

        FrontLeft.setPower(lpow);
        BackLeft.setPower(lpow);
        FrontRight.setPower(rpow);
        BackRight.setPower(rpow);
        Middle.setPower(mpow);
    }

}
