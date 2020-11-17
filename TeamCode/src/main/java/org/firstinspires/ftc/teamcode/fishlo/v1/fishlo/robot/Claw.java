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

    boolean armIsUp = true;
    boolean clawIsClosed = true;

    public Claw(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        claw = robot.hardwareMap.servo.get("claw");
        arm = robot.hardwareMap.servo.get("arm");
    }

    @Override
    public void handle() {
        if (robot.gamepad1.x && !armIsUp) {
            if (arm.getPosition() == ARM_HOME) {
                armUp();
                armIsUp = true;
            }
        }
        else if (robot.gamepad1.x && armIsUp) {
            if (arm.getPosition() == ARM_MAX) {
                armDown();
                armIsUp = false;
            }
        }
        if (robot.gamepad1.b && !clawIsClosed) {
            if (claw.getPosition() == CLAW_HOME) {
                close();
                clawIsClosed = true;
            }
        }
        else if (robot.gamepad1.b && clawIsClosed) {
            if (claw.getPosition() == CLAW_MAX) {
                open();
                clawIsClosed = false;
            }
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
