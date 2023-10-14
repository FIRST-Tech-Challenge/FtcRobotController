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
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
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
        leftFront.setTargetPosition(500);
        leftBack.setTargetPosition(500);
        rightFront.setTargetPosition(500);
        rightBack.setTargetPosition(500);
    }
}
