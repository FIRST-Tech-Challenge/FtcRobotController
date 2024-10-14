package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

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
@Autonomous (name = "Move Practice" , group = "Autonomous Pathing Tuning")
public class MovePractice extends OpMode {
    private Telemetry telemetryA;

    public static double DISTANCE = 40;

    private boolean forward = true;

    private Follower follower;

    private Path move1;
    private Path move2;
    private Path move3;
    private Path move4;
    private Path move5;
    private Path move6;
    private Path move7;
    private Path move8;

    // TODO: adjust this for each auto
    private Pose startPose = new Pose(0, 0, 0);

    private int pathState;

    private ElapsedTime pausetime = new ElapsedTime();


    /**
     * This initializes the Follower and creates the forward and backward Paths. Additionally, this
     * initializes the FTC Dashboard telemetry.
     */
    @Override
    public void init() {
        follower = new Follower(hardwareMap);

        move1 = new Path(new BezierLine(new Point(0,0, Point.CARTESIAN), new Point(-17,0, Point.CARTESIAN)));
        move1.setConstantHeadingInterpolation(0);
        move2 = new Path(new BezierLine(new Point(-17,0, Point.CARTESIAN), new Point(-17,5, Point.CARTESIAN)));
        move2.setConstantHeadingInterpolation(0);
       // backwards.setReversed(true);
        follower.followPath(move1);

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("This will run the robot in a straight line going " + DISTANCE
                            + " inches forward. The robot will go forward and backward continuously"
                            + " along the path. Make sure you have enough room.");
        telemetryA.update();
        pathState = 10;
        follower.setStartingPose(startPose);
        follower.followPath(move1);
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
            case 2:
                follower.followPath();

            case 10:
                if (pausetime.seconds() > 2 ) {
                    follower.followPath(move2);
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
        }
        telemetryA.addData("going forward", forward);
        follower.telemetryDebug(telemetryA);
    }
}
