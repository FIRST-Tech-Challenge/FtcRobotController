package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Claw extends SubSystem {
    private Servo claw;
    private DcMotor arm;

    public static final double CLAW_HOME = 0;
    public static final double CLAW_MAX = 0.7;
    public static double claw_speed = 0.5;


    public Claw(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        claw = robot.hardwareMap.servo.get("claw");
        arm = robot.hardwareMap.dcMotor.get("arm");
        close();
    }

    @Override
    public void handle() {
        claw_speed = (robot.gamepad2.left_stick_x < 0) ? Math.pow(robot.gamepad2.left_stick_x, 2) : -Math.pow(robot.gamepad2.left_stick_x, 2);
        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        arm.setPower(claw_speed);
        if (robot.gamepad2.x) {
            open();
        }
        else if (robot.gamepad2.b) {
            close();
        }

    }

    @Override
    public void stop() {
        open();
    }

    public void close() {
        claw.setPosition(CLAW_MAX);
    }

    public void open() {
        claw.setPosition(CLAW_HOME);
    }

    ElapsedTime armTimer = new ElapsedTime();

    public void armDown() {
        armTimer.reset();
        while (armTimer.milliseconds() <= 500) {
            arm.setPower(-1);
        }
        arm.setPower(0);
    }

    public void armUp() {
        armTimer.reset();
        while (armTimer.milliseconds() <= 1000) {
            arm.setPower(.3);
        }
        arm.setPower(0);
    }



}
