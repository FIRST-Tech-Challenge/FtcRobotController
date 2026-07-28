package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.common.autonomous.AutoSequence;
import org.firstinspires.ftc.teamcode.common.autonomous.TimedDriveStep;
import org.firstinspires.ftc.teamcode.common.autonomous.TimedIntakeStep;
import org.firstinspires.ftc.teamcode.common.autonomous.WaitStep;
import org.firstinspires.ftc.teamcode.robots.teamA.TeamARobot;

/** Team A's cautious non-blocking autonomous demonstration. */
@Autonomous(name = "Team A Autonomous Demo", group = "Team A")
public class TeamAAutoOpMode extends OpMode {
    private static final double DRIVE_FORWARD_POWER = 0.20;
    private static final double DRIVE_DURATION_SECONDS = 0.75;
    private static final double INTAKE_DURATION_SECONDS = 0.50;
    private static final double FINAL_WAIT_SECONDS = 0.25;

    private TeamARobot robot;
    private AutoSequence autoSequence;

    @Override
    public void init() {
        robot = new TeamARobot();
        robot.initialize(hardwareMap);
        autoSequence = new AutoSequence(
                new TimedDriveStep(robot, DRIVE_FORWARD_POWER, 0.0, 0.0,
                        DRIVE_DURATION_SECONDS),
                new TimedIntakeStep(robot, INTAKE_DURATION_SECONDS),
                new WaitStep(FINAL_WAIT_SECONDS));

        telemetry.addData("Status", "Team A autonomous initialized");
        telemetry.update();
    }

    @Override
    public void start() {
        robot.disableDrive();
        robot.stopIntake();
        autoSequence.start();
    }

    @Override
    public void loop() {
        autoSequence.update();
        if (autoSequence.isFinished()) {
            robot.disableDrive();
            robot.stopIntake();
        }

        robot.update();
        publishTelemetry();
    }

    @Override
    public void stop() {
        if (autoSequence != null) {
            autoSequence.stop();
        }
        if (robot != null) {
            robot.stop();
        }
    }

    private void publishTelemetry() {
        telemetry.addData("Auto Step", autoSequence.getCurrentStepName());
        telemetry.addData("Sequence Complete", autoSequence.isFinished());
        telemetry.addData("Drive State", robot.getDriveStateName());
        telemetry.addData("Intake State", robot.getIntakeStateName());
        telemetry.addData("Vision State", robot.getVisionStateName());
        telemetry.update();
    }
}
