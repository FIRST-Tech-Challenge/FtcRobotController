package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.paths.Path;
import org.firstinspires.ftc.teamcode.follower.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.follower.api.Paths.*;

@Autonomous(name = "Crazy")
public class ExampleAuto extends LinearOpMode {

    private enum State {
        DETECT,
        WAIT_DETECT,
        DELAY_DETECT,

        TOPRIGHT,
        WAIT_TOPRIGHT,
        DELAY_TOPRIGHT,

        MID_OUT,
        WAIT_MID_OUT,
        DELAY_MID_OUT,

        DONE,
        FAILED
    }

    private State robotstate = State.DETECT;
    TelemetryManager telemetryM;

    private Follower follower;
    private Timer timer;

    private static final double PATH_TIMEOUT = 5.0;
    private static final double STATE_DELAY = 0.2;

    private boolean timedOut = false;
    private String failedPath = "None";

    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start = p.of(0, 0, 90);
    private final Pose topRight = p.of(24, 24, 90);
    private final Pose rotate = p.of(24, 0, 90);
    private final Pose midOut = p.of(0, 24, 180);

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        timer = new Timer();


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            follower.update();

            switch (robotstate) {

                case DETECT:
                    follower.follow(topR());
                    newState(State.WAIT_DETECT);
                    break;

                case WAIT_DETECT:
                    if (!follower.isBusy()) {
                        newState(State.DELAY_DETECT);
                    } else if (timer.get(TimeUnit.SECONDS) >= PATH_TIMEOUT) {
                        fail("start -> topRight");
                    }
                    break;

                case DELAY_DETECT:
                    if (timer.get(TimeUnit.SECONDS) >= STATE_DELAY) {
                        newState(State.TOPRIGHT);
                    }
                    break;

                case TOPRIGHT:
                    follower.follow(topRightPath());
                    newState(State.WAIT_TOPRIGHT);
                    break;

                case WAIT_TOPRIGHT:
                    if (!follower.isBusy()) {
                        newState(State.DELAY_TOPRIGHT);
                    } else if (timer.get(TimeUnit.SECONDS) >= PATH_TIMEOUT) {
                        fail("topRight -> rotate");
                    }
                    break;

                case DELAY_TOPRIGHT:
                    if (timer.get(TimeUnit.SECONDS) >= STATE_DELAY) {
                        newState(State.MID_OUT);
                    }
                    break;

                case MID_OUT:
                    follower.follow(midOutPath());
                    newState(State.WAIT_MID_OUT);
                    break;

                case WAIT_MID_OUT:
                    if (!follower.isBusy()) {
                        newState(State.DELAY_MID_OUT);
                    } else if (timer.get(TimeUnit.SECONDS) >= PATH_TIMEOUT) {
                        fail("rotate -> midOut");
                    }
                    break;

                case DELAY_MID_OUT:
                    if (timer.get(TimeUnit.SECONDS) >= STATE_DELAY) {
                        newState(State.DONE);
                    }
                    break;

                case DONE:
                    follower.manual(0, 0, 0);
                    break;


            }
            Pose current = follower.pose();

            telemetryM.addData("X", current.x());
            telemetryM.addData("Y", current.y());
            telemetryM.addData("Heading", current.heading());
            telemetryM.addData("State", robotstate);
            telemetryM.addData("Follower Busy", follower.isBusy());
            telemetryM.addData("Timer (s)", timer.get(TimeUnit.SECONDS));
            telemetryM.addData("Timed Out", timedOut);
            telemetryM.addData("Failed Path", failedPath);
            telemetryM.addData("Distance To Path End", follower.distanceToEndpoint());
            telemetryM.update();
        }
    }

    private Path topR() {
        return line(start, topRight).constant(topRight);
    }

    private Path topRightPath() {
        return line(topRight, rotate).constant(rotate);
    }

    private Path midOutPath() {
        return line(rotate, midOut).constant(midOut);
    }

    private void newState(State state) {
        robotstate = state;
        timer.reset();
    }

    private void fail(String pathName) {
        timedOut = true;
        failedPath = pathName;
        newState(State.DONE);
    }
}