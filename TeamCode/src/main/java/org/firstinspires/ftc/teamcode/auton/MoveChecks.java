package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

/**
 * This is the StraightBackAndForth autonomous OpMode. It runs the robot in a specified distance
 * straight forward. On reaching the end of the forward Path, the robot runs the backward Path the
 * same distance back to the start. Rinse and repeat! This is good for testing a variety of Vectors,
 * like the drive Vector, the translational Vector, and the heading Vector. Remember to test your
 * tunings on CurvedBackAndForth as well, since tunings that work well for straight lines might
 * have issues going in curves.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/12/2024
 */
@Config
@Autonomous (name = "B Move Checks" , group = "Autonomous Pathing Tuning")
public class MoveChecks extends OpMode {
    AutonPaths1 autonp = new AutonPaths1(this);
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(7.25, 89.25, 0);
    private Pose spinPose = new Pose(-31, 9, 3.14);

    private int pathState;

    private ElapsedTime pausetime = new ElapsedTime();


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        autonp.init();
        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("I am " +
                this);
        telemetryA.addLine("This is our first trial moves for Into The Deep competition.");
        telemetryA.update();
        pathState = 1;
        follower.setStartingPose(startPose);
        follower.followPath(autonp.move1);
    }

    /**
     * This runs the OpMode, updating the Follower as well as printing out the debug statements to
     * the Telemetry, as well as the FTC Dashboard.
     */
    @Override
    public void loop() {
        switch (pathState) {
            case 1: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pausetime.reset();
                    pathState = 10;
                }
                break;
            case 10:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(autonp.move2);
                    pathState = 11;
                }
                break;
            case 11: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 20;
                }
                break;
            case 20:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(autonp.move3);
                    pathState = 21;
                }
                break;
            case 21: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 30;
                }
                break;
            case 30:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(autonp.move4);
                    pathState = 31;
                }
                break;
            case 31: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 40;
                }
                break;
            case 40:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(autonp.move5);
                    pathState = 41;
                }
                break;
            case 41: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 50;
                }
                break;
            case 50:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(autonp.move6);
                    pathState = 51;
                }
                break;
            case 51: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 60;
                }
                break;
            case 60:
                if (pausetime.seconds() > 0.5 ) {
                    follower.followPath(autonp.move7);
                    pathState = 61;
                }
                break;
            case 61: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 70;
                }
                break;
            case 70:
                if (pausetime.seconds() > 0.5 ) {
                    follower.setStartingPose(new Pose(-10, -35, -0.85));
                    follower.followPath(autonp.move8);
                    pathState = 71;
                }
                break;
            case 71: // starts following the first path to score on the spike mark
                follower.update();
                if (!follower.isBusy()) {
                    pathState = 100;
                }
                break;
            case 100:
                break;
        }
        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
