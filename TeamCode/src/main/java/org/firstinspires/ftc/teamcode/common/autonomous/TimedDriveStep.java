package org.firstinspires.ftc.teamcode.common.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Requests fixed robot-relative drive values for a non-blocking duration. */
public class TimedDriveStep implements AutoStep {
    private final AutonomousRobotControl robot;
    private final double forward;
    private final double strafe;
    private final double rotate;
    private final double durationSeconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started;

    public TimedDriveStep(AutonomousRobotControl robot, double forward, double strafe,
            double rotate, double durationSeconds) {
        if (robot == null) {
            throw new IllegalArgumentException("A timed drive step needs robot control.");
        }
        WaitStep.validateDuration(durationSeconds);
        this.robot = robot;
        this.forward = forward;
        this.strafe = strafe;
        this.rotate = rotate;
        this.durationSeconds = durationSeconds;
    }

    @Override public void start() { timer.reset(); started = true; robot.drive(forward, strafe, rotate); }
    @Override public void update() { if (!isFinished()) { robot.drive(forward, strafe, rotate); } }
    @Override public boolean isFinished() { return started && timer.seconds() >= durationSeconds; }
    @Override public void stop() { robot.disableDrive(); }
    @Override public String getName() { return "TimedDrive"; }
}
