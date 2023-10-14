package org.firstinspires.ftc.teamcode.robots.csbot.rr_drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Motor Tick Test", group = "test")
public class MotorTickTest extends OpMode {
    public DcMotorEx leftFront, leftBack, rightBack, rightFront;
    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
        leftBack = hardwareMap.get(DcMotorEx.class, "motorBackLeft");
        rightBack = hardwareMap.get(DcMotorEx.class, "motorBackRight");
        rightFront = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setTargetPosition(0);
        leftBack.setTargetPosition(0);
        rightFront.setTargetPosition(0);
        rightBack.setTargetPosition(0);
        leftFront.setPower(.25);
        leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFront.setPower(.25);
        rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBack.setPower(.25);
        leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBack.setPower(.25);
        rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    @Override
    public void loop() {
        telemetry.addData("Left Front Motor Position\t", leftFront.getCurrentPosition());
        telemetry.addData("Left Back Motor Position\t", leftBack.getCurrentPosition());
        telemetry.addData("Right Front Motor Position\t", rightFront.getCurrentPosition());
        telemetry.addData("Right Back Motor Position\t", rightBack.getCurrentPosition());
        leftFront.setTargetPosition(2000);
        leftBack.setTargetPosition(2000);
        rightFront.setTargetPosition(2000);
        rightBack.setTargetPosition(2000);
    }
}
