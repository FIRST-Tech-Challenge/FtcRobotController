package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.bots.AutomationBot;
import org.firstinspires.ftc.teamcode.bots.HangBot;
import org.firstinspires.ftc.teamcode.bots.LimelightBot;

import org.firstinspires.ftc.teamcode.bots.OdometryBot;
import org.firstinspires.ftc.teamcode.pedroPathing.follower.*;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer;

public class PedroAuto {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private final Pose startPose = new Pose(7.698663426488457, 53.890643985419196, 0);// starting position of robot
    private final Pose scoreSpecimen = new Pose(40, 66, Math.toRadians(180));// position where specimen is scored on submersible, robot is aligned to submerisble with back facing it

//    private final Pose sample1 = new Pose(35, 23,0); //these three poses are just behind the samples
    private final Pose samplePivot = new Pose(35, 12.7,0); //pivot from one point to grab all 3 samples
//    private final Pose sample3 = new Pose(35, 6,0);

    /** Park Pose for our robot, after we do all of the scoring. */
    private final Pose parkPose = new Pose(60, 46, Math.toRadians(90));

    /** coordinate to control bezier curve for parking, to go around the submersible must use bezier curve, this is mid point.*/
    private final Pose parkControl = new Pose (37, 25, 0);

    private Path scorePreload, park;

    private PathChain pickup1, pickup2, pickup3, score1, score2, score3;

    public void buildPaths(){
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(scoreSpecimen)));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scoreSpecimen.getHeading());

        pickup1 = follower.pathBuilder()
                .addPath(new BezierLine(new));

    }






}
