package org.firstinspires.ftc.teamcode.fishlo.v1.fishlo.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.SubSystem;

public class Shooter extends SubSystem {

    private Servo pusher;
    private DcMotor shooter;

    public static final double PUSHER_HOME = 0;
    public static final double PUSHER_MAX = 0.5;
    public static final double SHOOTER_SPEED = 0.5;
    boolean shooter_started = false;
    ElapsedTime shooterTimer = new ElapsedTime();

    public Shooter(Robot robot) {
        super(robot);
    }

    @Override
    public void init() {
        shooter = robot.hardwareMap.dcMotor.get("shooter");
    }

    @Override
    public void handle() {
        if (robot.gamepad2.right_bumper){
            startShooter();
        }
        else if (robot.gamepad2.left_bumper) {
            stopShooter();
        }
        if (robot.gamepad2.a && shooter_started) {
            shoot();
            shooterTimer.reset();
            if (shooterTimer.seconds() > 0.5){
                resetPusher();
            }
        }
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
        shooter_started = true;
    }

    public void stopShooter() {
        shooter.setPower(0);
        shooter_started = false;
    }

    public void resetPusher() {
        pusher.setPosition(PUSHER_HOME);
    }
}
