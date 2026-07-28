package org.firstinspires.ftc.teamcode.common.autonomous;

import com.qualcomm.robotcore.util.ElapsedTime;

/** Requests forward intake behavior for a non-blocking duration. */
public class TimedIntakeStep implements AutoStep {
    private final AutonomousRobotControl robot;
    private final double durationSeconds;
    private final ElapsedTime timer = new ElapsedTime();
    private boolean started;

    public TimedIntakeStep(AutonomousRobotControl robot, double durationSeconds) {
        if (robot == null) {
            throw new IllegalArgumentException("A timed intake step needs robot control.");
        }
        WaitStep.validateDuration(durationSeconds);
        this.robot = robot;
        this.durationSeconds = durationSeconds;
    }

    @Override public void start() { timer.reset(); started = true; robot.startIntake(); }
    @Override public void update() { if (!isFinished()) { robot.startIntake(); } }
    @Override public boolean isFinished() { return started && timer.seconds() >= durationSeconds; }
    @Override public void stop() { robot.stopIntake(); }
    @Override public String getName() { return "TimedIntake"; }
}
