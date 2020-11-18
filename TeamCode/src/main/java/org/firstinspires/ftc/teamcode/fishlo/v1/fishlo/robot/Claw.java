package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Claw extends SubSystem {
    private Servo claw;
    private Servo arm;

    public static final double CLAW_HOME = 0;
    public static final double CLAW_MAX = 0.5;
    public static final double ARM_HOME = 0.3;
    public static final double ARM_MAX = 0;

    boolean armIsUp;
    boolean clawIsClosed;

    public Claw(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        claw = robot.hardwareMap.servo.get("claw");
        arm = robot.hardwareMap.servo.get("arm");
        armUp();
        open();
    }

    @Override
    public void handle() {
        if (robot.gamepad1.x) {
            open();
        }
        else if (robot.gamepad1.b) {
            close();
        }
        if (robot.gamepad1.a) {
            armUp();
        }
        else if (robot.gamepad1.y) {
            armDown();
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

    public void armDown() {arm.setPosition(ARM_MAX);}

    public void armUp() {arm.setPosition(ARM_HOME);}

}
