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
//    double closeEnoughDistance = Math.PI/3; // originally Math.Pi/5
//    double closeEnoughAngle = 40;// originallly 30

    double closeEnoughDistance = 10;
    double closeEnoughAngle = Math.PI/5;
    public PathPoint targetPose;
    volatile boolean running = true;

    // constructor stuff
    Mecanum drivetrain;
    Telemetry telemetry;
    GivesPosition odometry;
    String pathFile;
    Thread loop;

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
    public Follower(Mecanum drivetrain, GivesPosition odometry, String pathFile, Telemetry telemetry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.telemetry = telemetry;
        this.pathFile = pathFile;

//        loop = new Thread(() -> {

            Path path = ImportPath.getPath(pathFile);


            //index of current target point
            int i = 0;
             telemetry.addData("I am running in the loop thread ", "yes I am");
//            muthaloop:

            while (i < path.size() && running) {
                telemetry.addData("In the loop", "mutha loop");

                pose position = odometry.getPosition();

                //if an unvisited point is close enough, move towards it
                for (int j = i + 1; j < path.size(); j++) {
                    if (path.get(j).distTo(new point(position.x, position.y)) < closeEnoughDistance) {
                        i = j;
                        break;
                    }

                }
                double distToLast = path.get(path.size() - 1).distTo(new point(position.x, position.y));
                double angleToLast = Math.abs(RotationUtil.turnLeftOrRight(position.r, path.get(path.size() - 1).dir, Math.PI / 2));

                if (distToLast < closeEnoughDistance && angleToLast < closeEnoughAngle) {
//                    if last point is reached, end
                    break;
                }


                //move robot towards i
                PathPoint target = path.get(i);

                telemetry.addData("Destination", String.format("%.1f %.1f %.1f", target.x, target.y, target.dir));

                //diff in rotation
                double rotDiff = RotationUtil.turnLeftOrRight(position.r, target.dir, Math.PI * 2);

                //diff in translation
                point transDiff = new point(target.x - position.x, target.y - position.y);
                transDiff.scale(target.speed);

                //convert from field angles to robot intrinsic angles
                point transDiffIntrinsic = transDiff.rotate(position.r - Math.PI / 2);

                //actually move
                drivetrain.drive(transDiffIntrinsic.x / 90, transDiffIntrinsic.y / 90, rotDiff / 90);

//                telemetry.addData("driving velocity",
//                        transDiffIntrinsic.x + " " + transDiffIntrinsic.y + " " + rotDiff);
                telemetry.addData("Odometry Position", odometry.getPosition());
                telemetry.addData("dist to last", distToLast + " " + angleToLast);

                telemetry.update();
            }
            drivetrain.drive(0, 0, 0);
            telemetry.addData("Done with path", "done");
            telemetry.update();
//        });
    }

    public void start(){
        running = true;
    }

    public void stop(){
        running = false;

    }

}
