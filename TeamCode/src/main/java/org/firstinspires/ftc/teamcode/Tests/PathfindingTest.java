package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utils.LinearOpMode2026;
import org.firstinspires.ftc.teamcode.Utils.MatrixDrawer;
import org.firstinspires.ftc.teamcode.Utils.Movement.PPRPHybrid;
import org.firstinspires.ftc.teamcode.Utils.Movement.Pathfinder;
import org.firstinspires.ftc.teamcode.Utils.Movement.Position;
import org.firstinspires.ftc.teamcode.Utils.Threads.FollowerThread;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Random;

@TeleOp
public class PathfindingTest extends LinearOpMode2026 {

    public Follower follower;
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(9, 9, 90);
    private final Pose path1 = poseFactory.of(13, 13, 90);

    @Override
    public void runOpMode() throws InterruptedException {
        config();
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(9,9,Math.toRadians(90)));
        follower.update();
        FollowerThread folThread = new FollowerThread(this,follower);
        PPRPHybrid.setFollower(follower);
        //For starter bot
        //mot[1].setDirection(DcMotorSimple.Direction.FORWARD); mot[3].setDirection(DcMotorSimple.Direction.FORWARD);
        //posGet.setSetup(RobotOdoSetup.PINPOINTSTARTER2026);
        Random random = new Random();
        //PosiBasedDriveThread dt = new PosiBasedDriveThread(this);
        Pathfinder.compileDefaultObstaclesLC(11);
        MatrixDrawer.fillSquare(Pathfinder.obstacle,38,0,120,72,true);
        //posGet.setOffset(9,9);
        waitForStart();
        folThread.start();
        //dt.start();
        Position[] startLaunch = new Position[] {
                new Position(9,9,0),
                new Position(10,10,0),
                new Position(11,11,0),
                new Position(12,12,0),
                new Position(13,13,0)
        };
        Position[] startLaunch2 = new Position[] {
                new Position(13,13,0),
                new Position(13,14,0),
                new Position(13,15,0),
                new Position(14,16,0),
                new Position(14,17,0)
        };
        PPRPHybrid.runPath(startLaunch);
        //follower.follow(Paths.line(start, path1).constant(path1));
        PPRPHybrid.lastPose = path1;
        //dt.queuePath(startLaunch);
        sleep(50);
        //while(!dt.idle());
        while(follower.isBusy()) {}
        //follower.hold(path1);
        //PPRPHybrid.runPath(startLaunch2);
        //sleep(50);

        int runs = 0;

        while(opModeIsActive()) {
            int x = -1; int y = -1;
            for(int i = 0; i < 30; i++) {
                int x0 = random.nextInt(120)+10;
                int y0 = random.nextInt(120)+10;
                if(!Pathfinder.obstacle[x0][y0]) {
                    x = x0;
                    y = y0;
                    break;
                }
            }
            if(x > 0) {
                Pathfinder.pathfind(
                        (int) PPRPHybrid.lastPose.x(),
                        (int) PPRPHybrid.lastPose.y(),
                        (int) (90-Math.toDegrees(PPRPHybrid.lastPose.heading())),
                        x,
                        y
                );
                telemetry.addData("Current pos",PPRPHybrid.lastPose.toString());
                telemetry.addData("Target pos", x+","+y);
                Pathfinder.saveI(1,50,"PathfinderTest"+runs);
                //dt.queuePath(Pathfinder.pos.toArray(new Position[0]));
                if(Pathfinder.pos.size() < 2) continue;
                PPRPHybrid.runPath(Pathfinder.pos.toArray(new Position[0]));
                sleep(40);
                //while(!dt.idle() && opModeIsActive() && !gamepad1.a);
                while(follower.isBusy() && opModeIsActive() && !gamepad1.a){
                    telemetry.addData("Position",follower.pose().toString());
                    telemetry.addData("Follower Status",follower.mode().toString());
                    telemetry.addData("Target Pos",PPRPHybrid.lastPose.toString());
                    telemetry.update();
                }
                follower.hold(PPRPHybrid.lastPose);
                while(gamepad1.a);
                runs++;
            }

        }
    }
}
