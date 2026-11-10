package org.firstinspires.ftc.teamcode.mechanisims;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LaunchSystem {
    //The feeder servos run this long when a shot is requested.
    private final static double FEED_TIME_SECONDS = 0.35;

    //After feeding a ball, the servos reverse to ensure there isn't a preloaded ball.
    private final static double REVERSE_FEED_SECONDS = 0.2;
    private final static double STOP_SPEED = 0.0;
    private final static double FULL_SPEED = 1.0;

    private final static double LAUNCHER_MOTOR_TIMEOUT = 3;
    private final static double LAUNCHER_TARGET_VELOCITY = 1125;

    private final DcMotorEx launcher;
    private final CRServo leftFeeder;
    private final CRServo rightFeeder;


    private final ElapsedTime launcherTimer = new ElapsedTime(); // Always is time since last launch initiated

    public enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCHING,
        REVERSE_FEED,
        WAITING_FOR_INTERVAL
    }

    private LaunchState launchState = LaunchState.IDLE;

    private double launchInterval = 0.0;

    public LaunchSystem(HardwareMap hardwareMap, String launcherName, String leftFeederName, String rightFeederName) {
        launcher = hardwareMap.get(DcMotorEx.class, launcherName);
        leftFeeder = hardwareMap.get(CRServo.class, leftFeederName);
        rightFeeder = hardwareMap.get(CRServo.class, rightFeederName);

        launcher.setZeroPowerBehavior(BRAKE);
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

    }

    /**
     * Initiates a launch cycle if ready (previous launch complete, including launch interval time).
     * Always succeeds if isReady() returns true.
     */
    public void requestLaunch() {
        if (isReady()) {
            launchState = LaunchState.SPIN_UP;
        }
    }

    public void update(double launchSpeedMultiplier) {
        switch (launchState) {
            case IDLE:
                if (launcherTimer.seconds() >= LAUNCHER_MOTOR_TIMEOUT) {
                    // Stop the launcher after some idle time so it isn't turning while loading.
                    launcher.setVelocity(STOP_SPEED);
                }
                break;
            case SPIN_UP:
                double targetLaunchVelocity = LAUNCHER_TARGET_VELOCITY * launchSpeedMultiplier;
                launcher.setVelocity(targetLaunchVelocity);
                if (launcher.getVelocity() >= targetLaunchVelocity) {
                    // initiate launch
                    leftFeeder.setPower(FULL_SPEED);
                    rightFeeder.setPower(FULL_SPEED);
                    launcherTimer.reset();
                    launchState = LaunchState.LAUNCHING;
                }
                break;
            case LAUNCHING:
                if (launcherTimer.seconds() >= FEED_TIME_SECONDS) {
                    launchState = LaunchState.REVERSE_FEED;
                    launcherTimer.reset();
                }
                break;
            case REVERSE_FEED:
                leftFeeder.setPower(-FULL_SPEED);
                rightFeeder.setPower(-FULL_SPEED);
                if (launcherTimer.seconds() >= REVERSE_FEED_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    launchState = LaunchState.WAITING_FOR_INTERVAL;
                }
                break;
            case WAITING_FOR_INTERVAL:
                if (launcherTimer.seconds() >= launchInterval) {
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }

    public double getLaunchInterval() {
        return launchInterval;
    }

    public void setLaunchInterval(double launchInterval) {
        this.launchInterval = launchInterval;
    }

    public void stopMotorIfIdle() {
        if (launchState == LaunchState.IDLE) {
            launcher.setVelocity(STOP_SPEED);
        }
    }

    /**
     * @return true If ready for a new launch. i.e. If a previous shot has been made, the launch interval
     * time has passed, and the launch cycle is complete.
     */
    public boolean isReady() {
        return launchState == LaunchState.IDLE;
    }

    public void logStatus(Telemetry telemetry) {
        telemetry.addData("Launch state", launchState);
        telemetry.addData("Launch motor speed", launcher.getVelocity());
    }
}
