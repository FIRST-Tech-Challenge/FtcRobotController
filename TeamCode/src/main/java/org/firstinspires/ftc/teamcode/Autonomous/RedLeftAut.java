package org.firstinspires.ftc.teamcode.Autonomous;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static java.lang.Math.toRadians;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.purepursuit.Path;
import com.arcrobotics.ftclib.purepursuit.Waypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.EndWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.GeneralWaypoint;
import com.arcrobotics.ftclib.purepursuit.waypoints.StartWaypoint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.BradBot;

@Autonomous
@Config
public class RedLeftAut extends LinearOpMode {
    int bark = 1;
    public static double
    x1=-40,y1=-40,h1=-180,v1=0.5,w1=0.1,fR1=5,bf1=2,
    x2=-56,y2=-39,h2=-180,v2=0.4,w2=0.1,fR2=5,bf2=1,
    x3 = 40,y3 =-32,h3=-180,v3=0.4,w3=0.15,fR3=9,bf3=3,
    xx1 = -34 ,yy1 = -60, hh1 = -220, vv1 = 0.8, ww1 = 0.4, ffR1 = 5,
    xx2 = 15 ,yy2 = -60, hh2 = -175, vv2 = 0.8, ww2 = 0.4, ffR2=9,
            xx3 = 35 ,yy3 = -34, hh3 = -150, vv3 = 0.7, ww3 = 0.4, ffR3=9;


    @Override
    public void runOpMode() throws InterruptedException {
        BradBot robot = new BradBot(this, false);
        robot.roadrun.setPoseEstimate(new Pose2d(-40.5, -63, Math.toRadians(-90)));
        Path[] toSpike = new Path[3];
        Waypoint start = new StartWaypoint( new com.arcrobotics.ftclib.geometry.Pose2d(
                -40.5, -64, new Rotation2d(toRadians(-90))));
        toSpike[0] = new Path(start);
        toSpike[0].add(new EndWaypoint(-50, -41, toRadians(-120), 0.4, 0.2, 5, 2, toRadians(5)));
        toSpike[1] = new Path(start);
        toSpike[1].add(new EndWaypoint(-40, -45, toRadians(-91), 0.4, 0, 5, 2, toRadians(10)));
        toSpike[2] = new Path(start);
        toSpike[2].add(new GeneralWaypoint(-40, -50, toRadians(-90),0.4,0.3, 5));
        toSpike[2].add(new EndWaypoint(x1, y1, toRadians(h1), v1, w1, fR1, bf1, toRadians(10)));
        Path[] spikeToBackdrop = new Path[3];
        robot.dropServo(1);
        robot.setRight(false);
        robot.setBlue(false);
        robot.observeSpike();

        while (!isStarted() || isStopRequested()) {
            bark = robot.getSpikePos();
            telemetry.addData("pixel", bark);
            packet.put("pix", bark);
            robot.update();
        }
        while (!isStopRequested() && opModeIsActive()) {
            robot.queuer.queue(false, true);
            robot.upAuto();
            robot.purpurAuto();
            robot.queuer.addDelay(2);
            robot.followPPPath(toSpike[bark]);
            robot.queuer.addDelay(0.8);
            robot.dropAuto(0);
            Path preToStack = new Path();
//            if(bark==1){
                preToStack.add(new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
                preToStack.add(new EndWaypoint(x2,y2,toRadians(h2),v2,w2,fR2,bf2,toRadians(10)));
//            }
            robot.queuer.addDelay(0.6);
            robot.followPPPath(preToStack);
            robot.queuer.addDelay(.6);
            robot.resetAuto();
            robot.intakeAuto(5);
            robot.queuer.waitForFinish();
            Path stackToBack = new Path();
//            if(bark==1){
            if(bark==0){
                y3=-24;
                yy3=-29;
            }
            if(bark==1){
                y3=-31;
                yy3=-36;
            }
            if(bark==2){
                y3=-37;
                yy3=-42;
            }
                stackToBack.add(new StartWaypoint(new Translation2d(x2+2, y2)));
                stackToBack.add(new GeneralWaypoint(xx1, yy1,toRadians(hh1),vv1,ww1,ffR1));
                stackToBack.add(new GeneralWaypoint(xx2,yy2, toRadians(hh2),vv2,ww2,ffR2));
                stackToBack.add(new GeneralWaypoint(xx3,yy3,toRadians(hh3), vv3,ww3,ffR3));
                stackToBack.add(new EndWaypoint(x3,y3,toRadians(h3),v3,w3,fR3,bf3,toRadians(10)));
//            }
            robot.followPPPath(stackToBack);
            robot.grabAuto();
            robot.lowAuto();
            robot.drop();
            Path park = new Path();
//            if(bark==1){
                park.add(new StartWaypoint(new Translation2d(currentPose.getX(), currentPose.getY())));
                park.add(new EndWaypoint(35,-48,toRadians(-180),0.2,0,5,2,toRadians(10)));
//            }
            robot.followPPPath(park);
            robot.queuer.addDelay(0.7);
            robot.resetAuto();

            robot.update();
        }
    }
}
