package org.firstinspires.ftc.teamcode.pathFollow;

import android.telecom.TelecomManager;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GivesPosition;
import org.firstinspires.ftc.teamcode.movement.Mecanum;
import org.firstinspires.ftc.teamcode.odometry.Odometry;
import org.firstinspires.ftc.teamcode.pathgen.ImportPath;
import org.firstinspires.ftc.teamcode.pathgen.Path;
import org.firstinspires.ftc.teamcode.pathgen.PathPoint;
import org.firstinspires.ftc.teamcode.utility.RotationUtil;
import org.firstinspires.ftc.teamcode.utility.point;
import org.firstinspires.ftc.teamcode.utility.pose;
import org.firstinspires.ftc.teamcode.vuforia.VuMarkNav;

public class Follower {
    double closeEnoughDistance = 10;
    public PathPoint targetPose;
    /**
     * Follows the pathFile found in local directory
     * @param drivetrain a mecanum drivetrain
     * @param odometry an activated, running odometry to be used for positioning
     */
    public Follower(Mecanum drivetrain, Odometry odometry, Telemetry telemetry){
        this(drivetrain, odometry, "paths.txt", telemetry);
    }
    /**
     * Follows a path specified by the fileName
     * @param drivetrain a mecanum drivetrain
     * @param odometry an activated, running odometry to be used for positioning
     */
    public Follower(Mecanum drivetrain, GivesPosition odometry, String pathFile, Telemetry telemetry){
        Path path = ImportPath.getPath(pathFile);

        //index of current target point
        int i = 0;
        while(i < path.size()){
            pose position = odometry.getPosition();
            //if close enough advance
            for(int j = i+1; j < path.size(); j++) {
                if (path.get(j).distTo(new point(position.x, position.y)) < closeEnoughDistance) {
                    i = j;
                    break;
                }
            }

            //move robot towards i
            PathPoint target = path.get(i);

            telemetry.addData("Destination", String.format("%.1f %.1f %.1f", target.x, target.y, target.dir));

            double rotDiff = RotationUtil.turnLeftOrRight(position.r, target.dir, Math.PI * 2);

            point transDiff = new point(target.x - position.x, target.y - position.y);
            transDiff.scale(target.speed);
            point transDiffIntrinsic = transDiff.rotate(-position.r - Math.PI/2);
            //-90 degs because convert from field anges to robot angles
            drivetrain.drive(transDiffIntrinsic.x/90, transDiffIntrinsic.y/90, rotDiff/90);
            telemetry.addData("driving towards", String.valueOf(Math.signum(transDiffIntrinsic.x))+ Math.signum(transDiffIntrinsic.y));
        }
    }

}
