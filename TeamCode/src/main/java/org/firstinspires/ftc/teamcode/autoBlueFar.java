package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
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

@Autonomous(name = "autoBlueFar", group = "Autonomous")
public class autoBlueFar extends LinearOpMode {

    private final PoseFactory poseFactory = PoseFactory.degrees();
    public final Pose startBlueFar = poseFactory.of(83.5188, 133.5213, -90);
    private final Pose launchFlower = poseFactory.of(135.5383, 94.766, 0);
    private final Pose flowerLaunchStart = poseFactory.of(135.5383, 94.766, 0);
    private final Pose flowerLaunch = poseFactory.of(84.6532, 22.3029, 90);
    private final Pose flowerLaunchControl1 = poseFactory.of(138.4403, 30.3363, 0);
    private final Pose launchFlower2Start = poseFactory.of(84.6532, 22.3029, -90);
    private final Pose launchFlower2 = poseFactory.of(94.5991, 3.5844, -90);
    private final Pose flowerLaunch2Start = poseFactory.of(94.5991, 3.5844, 0);
    private final Pose flowerLaunch2 = poseFactory.of(80.0097, 22.2789, 90);
    private final Pose launchParkStart = poseFactory.of(82.6558, 28.512, 0);
    private final Pose launchPark = poseFactory.of(136.4274, 31.2004, 0);


    public Path launchToFlowerABFar() {
        return Paths.line(startBlueFar, launchFlower).linear(startBlueFar, launchFlower);
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

    public Command autoBlueFar() {
        return sequential(
                follow(follower, launchToFlowerABFar()),
                follow(follower, flowerToLaunch()),
                follow(follower, launchToFlower2()),
                follow(follower, flowerToLaunch2()),
                follow(follower, launchToPark())
        );
    }

    // if this doesnt work make this static
    public Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startBlueFar);
        schedule(follow(follower, launchToFlowerABFar()));
        schedule(follow(follower, flowerToLaunch()));
        schedule(follow(follower, launchToFlower2()));
        schedule(follow(follower, flowerToLaunch2()));
        schedule(follow(follower, launchToPark()));

        schedule(autoBlueFar());

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
