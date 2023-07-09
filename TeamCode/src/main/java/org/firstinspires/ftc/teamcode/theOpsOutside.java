package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;
import java.lang.invoke.MethodType;

@TeleOp(name = "theOpsOutside")
public class theOpsOutside extends OpMode {
    DcMotor LFMotor;
    DcMotor RFMotor;
    DcMotor LBMotor;
    DcMotor RBMotor;

    public void moveDrive(){
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;
        telemetry.addData("Turn", turn);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        telemetry.addData("theta", theta);
        telemetry.addData("power", power);



        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));



        double LF = power * cos/max - turn;
        double RF = power * sin/max + turn;
        double LB = power * sin/max - turn;
        double RB = power * cos/max + turn;

        if ((LF + Math.abs(turn)) > 1) {
            LF /= power + Math.abs(turn);
        }
        if ((RF + Math.abs(turn)) > 1) {
            RF /= power + Math.abs(turn);
        }
        if ((LB + Math.abs(turn)) > 1) {
            LB /= power + Math.abs(turn);
        }
        if ((RB + Math.abs(turn)) > 1) {
            RB /= power + Math.abs(turn);
        }



        LFMotor.setPower(LF);
        RFMotor.setPower(RF);
        LBMotor.setPower(LB);
        RBMotor.setPower(RB);

//        LFMotor.setPower(speed*v1);
//        RFMotor.setPower(speed*v2);
//        LBMotor.setPower(speed*v3);
//        RBMotor.setPower(speed*v4);
    }

    @Override
    public void init(){
        LFMotor = hardwareMap.get(DcMotor.class, "leftFront");
        RFMotor = hardwareMap.get(DcMotor.class, "rightFront");
        LBMotor = hardwareMap.get(DcMotor.class, "leftBack");
        RBMotor = hardwareMap.get(DcMotor.class, "rightBack");

        LFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RFMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RBMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        RFMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RBMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop() {
        moveDrive();

    }

}