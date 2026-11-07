
package org.firstinspires.ftc.teamcode;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Example Park Auto")
public class ParkAutoQual extends OpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Poses
    private final Pose startPose = poseFactory.of(8, 144, 270);
    private final Pose finalPose = poseFactory.of(8, 120, 270);

    // Path methods
    private Path startToFinal() {
        return line(startPose, finalPose).linear(startPose, finalPose);
    }

    // Autonomous routine
    private Command autoRoutine() {
        return sequential(
                follow(follower, startToFinal())
        );
    }

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        follower.setPose(startPose);
        follower.update();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();

        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );
        telemetry.addData("Follower Mode", follower.mode());

        telemetry.update();
    }

    public void stop() {

        // Automatically grabs and saves the final coordinates when the match ends
        if (follower != null) {
            OpModeStorage.autonomousEndPose = follower.pose();
        }
    }
}

