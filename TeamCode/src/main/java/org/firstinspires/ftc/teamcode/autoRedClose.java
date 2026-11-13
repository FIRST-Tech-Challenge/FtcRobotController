package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "autoRedClose", group = "Autonomous")
public class autoRedClose extends LinearOpMode {


    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startRedClose = poseFactory.of(59.3733, 10.1305, 90);
    private final Pose launchFlower = poseFactory.of(7.2179, 46.602, 180);
    private final Pose flowerLaunchStart = poseFactory.of(7.2179, 46.602, 180);
    private final Pose flowerLaunch = poseFactory.of(58.4508, 121.8493, -90);
    private final Pose flowerLaunchControl1 = poseFactory.of(18.0672, 118.2544, 0);
    private final Pose launchFlower2Start = poseFactory.of(58.4508, 121.8493, -90);
    private final Pose launchFlower2 = poseFactory.of(46.9955, 135.3328, 90);
    private final Pose flowerLaunch2Start = poseFactory.of(46.9955, 135.3328, 90);
    private final Pose flowerLaunch2 = poseFactory.of(58.8643, 121.4516, -90);
    private final Pose launchParkStart = poseFactory.of(58.8643, 121.4516, -90);
    private final Pose launchPark = poseFactory.of(3.8808, 106.7098, 0);

    public Path launchToFlower() {
        return Paths.line(startRedClose, launchFlower).linear(startRedClose, launchFlower);
    }

    public Path flowerToLaunch() {
        return Paths.curve(flowerLaunchStart, flowerLaunchControl1, flowerLaunch).linear(flowerLaunchStart, flowerLaunch);
    }

    public Path launchToFlower2() {
        return Paths.line(launchFlower2Start, launchFlower2).linear(launchFlower2Start, launchFlower2);
    }

    public Path flowerToLaunch2() {
        return Paths.line(flowerLaunch2Start, flowerLaunch2).linear(flowerLaunch2Start, flowerLaunch2);
    }

    public Path launchToPark() {
        return Paths.line(launchParkStart, launchPark).linear(launchParkStart, launchPark);
    }

    private Command autoRoutine() {
        return sequential(
                follow(follower, launchToFlower()),
                follow(follower, flowerToLaunch()),
                follow(follower, launchToFlower2()),
                follow(follower, flowerToLaunch2()),
                follow(follower, launchToPark())

        );
    }

    public static Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startRedClose);
        schedule(follow(follower, launchToFlower()));
        schedule(follow(follower, flowerToLaunch()));
        schedule(follow(follower, launchToFlower2()));
        schedule(follow(follower, flowerToLaunch2()));
        schedule(follow(follower, launchToPark()));


        schedule(autoRoutine());

        waitForStart();

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());
            telemetry.update();

        }
    }
}

