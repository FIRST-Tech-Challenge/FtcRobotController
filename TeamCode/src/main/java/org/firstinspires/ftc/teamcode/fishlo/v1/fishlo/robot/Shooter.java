package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Shooter extends SubSystem {

    private Servo pusher;
    private DcMotor shooter;

    public static final double PUSHER_HOME = 0;
    public static final double PUSHER_MAX = 0.5;
    public static final double SHOOTER_SPEED = 0.5;

    public Shooter(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        shooter = robot.hardwareMap.dcMotor.get("shooter");
    }

    @Override
    public void handle() {

    }

    @Override
    public void stop() {
        stopShooter();
        resetPusher();
    }

    public void shoot() {
        pusher.setPosition(PUSHER_MAX);
    }

    public void startShooter() {
        shooter.setPower(SHOOTER_SPEED);
    }

    public void stopShooter() {
        shooter.setPower(0);
    }

    public void resetPusher() {
        pusher.setPosition(PUSHER_HOME);
    }
}
