package org.firstinspires.ftc.teamcode.teamPedroPathing;

import static com.pedropathing.api.Paths.line;

import static org.lhssa.ftc.teamcode.pedroPathing.PedroTeleopData.startingPose;

import com.pedropathing.api.Paths.*;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import org.lhssa.ftc.teamcode.pedroPathing.AllianceColor;


public class TeamPoses {
    public static AllianceColor canonicalColor = AllianceColor.RED;
    public static Pose startShootPose = new Pose(56.0, 10.0,Math.toRadians(90));
    public static Pose moveAwayFromWall = new Pose(56.0, 12.0,Math.toRadians(90));
    public static Pose BlueParkTemp = new Pose(10.23, 31.4,Math.toRadians(90));
    public static Pose BlueStartTemp = new Pose(22.7, 132.5,Math.toRadians(-90));
    public static Pose startGardenPose = new Pose (17.0, 10.0, Math.toRadians(180));
    public static Pose endGardenPose = new Pose (5.0, 10.0, Math.toRadians(180));
    public static Pose parkPose = new Pose (12.400, 103.800, Math.toRadians(90));


}