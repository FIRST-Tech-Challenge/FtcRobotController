package org.firstinspires.ftc.teamcode.robots.reachRefactor.opModes;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.Constants;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.reachRefactor.subsystems.Robot;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.ExponentialSmoother;
import org.firstinspires.ftc.teamcode.robots.reachRefactor.utils.StickyGamepad;

/** Controls
 * Pregame
 * x - set alliance to blue
 * b - set alliance to red
 * a - toggle FTC dashboard
 * y - toggle drivetrain smoothing
 * left bumper - decrement state
 * right bumper - increment state
 *
 * Tele-Op
 * y - toggle drivetrain smoothing
 * right stick y - forward
 * left stick x - rotate
 *
 */
@TeleOp(name = "refactored FF_6832")
public class FF_6832 extends OpMode {
    private Robot robot;

    // state
    private boolean active;
    private int state;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    // timing
    private long lastLoopClockTime, loopTime;
    private double averageLoopTime;
    private ExponentialSmoother loopTimeSmoother;

    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        active = true;
        state = 0;

        lastLoopClockTime = System.nanoTime();
        loopTimeSmoother = new ExponentialSmoother(Constants.AVERAGE_LOOP_TIME_SMOOTHING_FACTOR);

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        robot = new Robot(hardwareMap, telemetry, Constants.DEFAULT_DASHBOARD_ENABLED);
    }

    private void handleStateSwitch() {
        if (!active) {
            if (stickyGamepad1.left_bumper)
                state -= 1;
            if (stickyGamepad1.right_bumper)
                state += 1;

            state %= Constants.GAME_STATES.length;
        }

        if (stickyGamepad1.start)
            active = !active;
    }

    private void handlePregameControls() {
        if(stickyGamepad1.x)
            robot.setAlliance(Constants.Alliance.BLUE);
        if(stickyGamepad1.b)
            robot.setAlliance(Constants.Alliance.RED);

        if(stickyGamepad1.a)
            robot.toggleIsDashboardEnabled();
        if(stickyGamepad1.y)
            robot.driveTrain.toggleSmoothingEnabled();
    }

    // Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
    @Override
    public void init_loop() {
        handleStateSwitch();

        if (active)
            handlePregameControls();

        update();
    }

    @Override
    public void start() {
        lastLoopClockTime = System.nanoTime();
    }

    private void handleTeleOp() {
        double forward = Math.pow(-gamepad1.right_stick_y, 3) * Constants.FORWARD_SCALING_FACTOR;
        double rotate = Math.pow(gamepad1.left_stick_x, 3) * Constants.ROTATE_SCALING_FACTOR;

        robot.driveTrain.drive(forward, rotate);

        if(stickyGamepad1.y)
            robot.driveTrain.toggleSmoothingEnabled();

        if(gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y)
            robot.stop();
    }

    @Override
    public void loop() {
        handleStateSwitch();

        if (active)
            switch(state) {
                case 0:
                    handleTeleOp();
                    break;
                case 1:
                    if (robot.getAlliance().equals(Constants.Alliance.RED)
                            && robot.articulate(Robot.Articulation.AUTONOMOUS_RED)) {
                        active = false;
                        state = 0;
                    } else if (robot.getAlliance().equals(Constants.Alliance.BLUE)
                            && robot.articulate(Robot.Articulation.AUTONOMOUS_BLUE)) {
                        active = false;
                        state = 0;
                    }
            }

        update();
    }

    private void handleTiming() {
        long loopClockTime = System.nanoTime();
        loopTime = loopClockTime - lastLoopClockTime;
        averageLoopTime = loopTimeSmoother.update(loopTime);
        lastLoopClockTime = loopClockTime;
    }

    private void update() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        handleTiming();

        robot.addTelemetryData("Active", active);
        robot.addTelemetryData("State", String.format("(%d): %s", state, Constants.GAME_STATES[state]));
        robot.addTelemetryData("Smoothing Enabled", robot.driveTrain.isSmoothingEnabled());
        robot.addTelemetryData("Dashboard Enabled", robot.isDashboardEnabled());
        robot.addTelemetryData("Average Loop Time", String.format("%d ms (%d hz)", (int) (averageLoopTime * 1e-6), (int) (1 / (averageLoopTime * 1e-9))));
        robot.addTelemetryData("Last Loop Time", String.format("%d ms (%d hz)", (int) (loopTime * 1e-6), (int) (1 / (loopTime * 1e-9))));

        robot.update();
    }
}
