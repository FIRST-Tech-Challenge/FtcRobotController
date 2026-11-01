package org.firstinspires.ftc.teamcode.pedroPathing.Paths;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import org.firstinspires.ftc.teamcode.follower.api.PoseFactory;
import org.firstinspires.ftc.teamcode.follower.follower.Follower;
import org.firstinspires.ftc.teamcode.follower.math.Pose;
import org.firstinspires.ftc.teamcode.follower.paths.Path;
import org.firstinspires.ftc.teamcode.follower.utils.Angle;
import org.firstinspires.ftc.teamcode.follower.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

import static org.firstinspires.ftc.teamcode.follower.api.Paths.*;

@Autonomous(name = "Crazy")
public class BlueAuto extends LinearOpMode {

    // =================================================================
    //  GLOBAL SETTINGS
    // =================================================================
    private static final double START_DELAY = 0.0;   // seconds to wait after START before leg 1

    // Defaults every leg starts with. Override any of them per leg in buildRoute().
    private static final double DEFAULT_PRE_DELAY       = 0.0;                 // s, wait before driving
    private static final double DEFAULT_POST_DELAY      = 0.2;                 // s, wait after arriving
    private static final double DEFAULT_TIMEOUT         = 5.0;                 // s, max time driving
    private static final double DEFAULT_POS_TOL         = 1.5;                 // in, "close enough"
    private static final double DEFAULT_HEADING_TOL_DEG = 5.0;                 // deg, "close enough"
    private static final double DEFAULT_TIMEOUT_POS_TOL = 4.0;                 // in, ok to continue at timeout

    // =================================================================
    //  POSES
    // =================================================================
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start            = p.of(85, 132, 270);
    private final Pose frontShoot       = p.of(85, 107, 270);
    private final Pose preIntake        = p.of(104.8, 133.6, 0);
    private final Pose postIntake       = p.of(0, 24, 180);
    private final Pose intFlower        = p.of(94, 10.6, 270);
    private final Pose firstFloControl  = p.of(77, 54, 0);
    private final Pose secondFloControl = p.of(94, 29.1, 0);
    private final Pose backShoot        = p.of(84.1, 33.5, 90);

    // =================================================================
    //  ACTIONS - what the robot does at the start / end of a leg
    // =================================================================
    private enum Action { NONE, SHOOT, INTAKE, STOP_INTAKE }

    private void startAction(Action a) {
        switch (a) {
            case SHOOT:       /* TODO: start shooter / feed rings */ break;
            case INTAKE:      /* TODO: intake on */                  break;
            case STOP_INTAKE: /* TODO: intake off */                 break;
            default: break;
        }
    }

    /** Return true when the action is finished so the post-delay can end early. */
    private boolean actionDone(Action a) {
        switch (a) {
            case SHOOT:  return false;   // TODO: return shooterFinished();
            case INTAKE: return false;   // TODO: return intakeFull();
            default:     return true;
        }
    }

    // =================================================================
    //  ROUTE - one line per leg. Change order, delays, tolerances here.
    // =================================================================
    private List<Leg> route;

    private void buildRoute() {
        route = new ArrayList<>();

        route.add(new Leg("Preload shot", line(start, frontShoot).constant(frontShoot), frontShoot)
                .postAction(Action.SHOOT).postDelay(1.5));

        route.add(new Leg("To preIntake", line(frontShoot, preIntake).linear(frontShoot, preIntake), preIntake)
                .duringAction(Action.INTAKE).postDelay(0.5));

        route.add(new Leg("Back to shoot", line(preIntake, frontShoot).linear(preIntake, frontShoot), frontShoot)
                .duringAction(Action.STOP_INTAKE).postAction(Action.SHOOT).postDelay(1.5));

        // The only curve
        route.add(new Leg("To intFlower",
                curve(frontShoot, firstFloControl, secondFloControl, intFlower).linear(frontShoot, intFlower),
                intFlower)
                .duringAction(Action.INTAKE).postDelay(0.5).timeout(6.0));

        route.add(new Leg("To backShoot", line(intFlower, backShoot).linear(intFlower, backShoot), backShoot)
                .duringAction(Action.STOP_INTAKE).postAction(Action.SHOOT).postDelay(1.5));

        route.add(new Leg("Park", line(backShoot, postIntake).linear(backShoot, postIntake), postIntake)
                .timeout(7.0).posTol(3.0));
    }

    // =================================================================
    //  LEG CONFIG (no need to edit below unless changing behavior)
    // =================================================================
    private static class Leg {
        final String name;
        final Path path;
        final Pose target;

        double preDelay       = DEFAULT_PRE_DELAY;
        double postDelay      = DEFAULT_POST_DELAY;
        double minPostDelay   = 0.0;      // post-delay can't end early before this
        double timeout        = DEFAULT_TIMEOUT;
        double posTol         = DEFAULT_POS_TOL;
        double headingTol     = Math.toRadians(DEFAULT_HEADING_TOL_DEG);
        double timeoutPosTol  = DEFAULT_TIMEOUT_POS_TOL;
        Action duringAction   = Action.NONE;   // started when the robot starts driving
        Action postAction     = Action.NONE;   // started when the robot arrives

        Leg(String name, Path path, Pose target) {
            this.name = name;
            this.path = path;
            this.target = target;
        }

        Leg preDelay(double s)        { preDelay = s; return this; }
        Leg postDelay(double s)       { postDelay = s; return this; }
        Leg minPostDelay(double s)    { minPostDelay = s; return this; }
        Leg timeout(double s)         { timeout = s; return this; }
        Leg posTol(double in)         { posTol = in; return this; }
        Leg headingTolDeg(double deg) { headingTol = Math.toRadians(deg); return this; }
        Leg timeoutPosTol(double in)  { timeoutPosTol = in; return this; }
        Leg duringAction(Action a)    { duringAction = a; return this; }
        Leg postAction(Action a)      { postAction = a; return this; }
    }

    // =================================================================
    //  STATE MACHINE
    // =================================================================
    private enum Phase { START_DELAY, PRE_DELAY, FOLLOW, WAIT, POST_DELAY, DONE }

    private Phase phase = Phase.START_DELAY;
    private int legIndex = 0;

    private TelemetryManager telemetryM;
    private Follower follower;
    private Timer timer;
    private boolean timedOut = false;
    private String failedPath = "None";
    private String arriveReason = "-";

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        timer = new Timer();

        buildRoute();

        waitForStart();
        if (isStopRequested()) return;

        setPhase(Phase.START_DELAY);

        while (opModeIsActive()) {

            follower.update();

            double t = timer.get(TimeUnit.SECONDS);
            Leg leg = phase == Phase.DONE ? null : route.get(legIndex);

            switch (phase) {

                case START_DELAY:
                    if (t >= START_DELAY) setPhase(Phase.PRE_DELAY);
                    break;

                case PRE_DELAY:
                    if (t >= leg.preDelay) setPhase(Phase.FOLLOW);
                    break;

                case FOLLOW:
                    follower.follow(leg.path);
                    startAction(leg.duringAction);
                    setPhase(Phase.WAIT);
                    break;

                case WAIT: {
                    double dist = follower.pose().distance(leg.target);
                    double headErr = Math.abs(Angle.error(follower.pose().heading(), leg.target.heading()));
                    boolean closeEnough = dist <= leg.posTol && headErr <= leg.headingTol;
                    boolean timeUp = t >= leg.timeout;

                    if (!follower.isBusy()) {
                        arrive(leg, "follower done");
                    } else if (closeEnough) {
                        arrive(leg, "close enough");              // fail-safe 1
                    } else if (timeUp && dist <= leg.timeoutPosTol) {
                        arrive(leg, "timeout, near target");      // fail-safe 2
                    } else if (timeUp) {
                        fail(leg.name);                           // fail-safe 3
                    }
                    break;
                }

                case POST_DELAY: {
                    boolean maxDone = t >= leg.postDelay;
                    boolean earlyDone = t >= leg.minPostDelay && leg.postAction != Action.NONE
                            && actionDone(leg.postAction);
                    if (maxDone || earlyDone) nextLeg();
                    break;
                }

                case DONE:
                    follower.manual(0, 0, 0);
                    break;
            }

            Pose current = follower.pose();
            telemetryM.addData("X", current.x());
            telemetryM.addData("Y", current.y());
            telemetryM.addData("Heading", Math.toDegrees(current.heading()));
            telemetryM.addData("Leg", leg == null ? "DONE" : (legIndex + 1) + "/" + route.size() + " " + leg.name);
            telemetryM.addData("Phase", phase);
            telemetryM.addData("Follower Busy", follower.isBusy());
            telemetryM.addData("Timer (s)", t);
            telemetryM.addData("Timed Out", timedOut);
            telemetryM.addData("Failed Path", failedPath);
            telemetryM.addData("Last Arrival", arriveReason);
            telemetryM.addData("Distance To Path End", follower.distanceToEndpoint());
            telemetryM.update();
        }
    }

    private void arrive(Leg leg, String reason) {
        arriveReason = leg.name + ": " + reason;
        startAction(leg.postAction);
        setPhase(Phase.POST_DELAY);
    }

    private void nextLeg() {
        legIndex++;
        setPhase(legIndex >= route.size() ? Phase.DONE : Phase.PRE_DELAY);
    }

    private void setPhase(Phase newPhase) {
        phase = newPhase;
        timer.reset();
    }

    private void fail(String pathName) {
        timedOut = true;
        failedPath = pathName;
        setPhase(Phase.DONE);
    }
}


