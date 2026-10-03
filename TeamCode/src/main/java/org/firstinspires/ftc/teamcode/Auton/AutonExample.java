package org.firstinspires.ftc.teamcode.Auton;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous
public class AutonExample extends OpMode {

    private Follower follower;

    private final PoseFactory pf = PoseFactory.degrees();//new posefactory thingy; it is supposed to help reduce the length of pose lines

    private final Pose startPose = pf.of(72, 72, 90);
    private final Pose pose1 = pf.of(120, 96, 0);

    private final Pose pose2 = pf.of(72, 24, 56);

    private final Pose pose3 = pf.of(72, 120, 90);

    private final Pose pose4 = new Pose(72, 48);

    private final Pose controlPose2 = new Pose(15, 86);

    private Path startPath() {
        return line(startPose, pose1).linear(startPose, pose1);
    }
    private Path path1(){
        return line(pose1, pose2).reverseTangent();
    }
    private Path path2(){
        return curve(pose2, controlPose2, pose3).linear(pose2, pose3);
    }
    private Path endPath(){
        return line(pose3, pose4).reverseTangent();
    }



    private Command autoRoutine() {
        return sequential(
                follow(follower, startPath()),
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, endPath())
        );
    }


    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    @Override
    public void start () {
        schedule(autoRoutine());

    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addLine("running fine  :)");
        telemetry.addLine("B-K, Have it your way, you rule!");

    }
}