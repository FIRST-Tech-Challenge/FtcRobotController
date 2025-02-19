package org.firstinspires.ftc.teamcode.JackBurr.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Config
public class LeftAutoV9 extends LinearOpMode {
    public static Pose startPose = new Pose(38, 62, Math.toRadians(270));
    public static Pose bucket = new Pose(50, 52, Math.toRadians(45));

    private Path scorePreload;

    @Override
    public void runOpMode() throws InterruptedException {
        scorePreload = new Path(new BezierLine(new Point(startPose), new Point(bucket)));
    }

}
