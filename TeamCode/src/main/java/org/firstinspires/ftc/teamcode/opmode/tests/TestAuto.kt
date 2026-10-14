package org.firstinspires.ftc.teamcode.opmode.tests

import com.pedropathing.api.Paths
import com.pedropathing.api.Paths.line
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.ivy.Command
import com.pedropathing.ivy.Scheduler
import com.pedropathing.ivy.Scheduler.schedule
import com.pedropathing.ivy.groups.Groups.sequential
import com.pedropathing.ivy.pedro.PedroCommands.follow
import com.pedropathing.math.Pose
import com.pedropathing.paths.Path
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.pedro.Constants

@Autonomous(name = "Test Autonomous", group = "Test")
class TestAuto: OpMode() {
    private lateinit var follower: Follower

    // Create waypoints along path
    private val poseFactory = PoseFactory.degrees()

    private val start = poseFactory.of(23.6633, 117.8144, 90.0)
    private val path1 = poseFactory.of(118.069, 117.8183, -26.3499)
    private val path1Control1 = poseFactory.of(32.2462, 117.6388, 0.0)
    private val path1Control2 = poseFactory.of(70.5491, 141.7883, 0.0)
    private val point2 = poseFactory.of(118.0429, 24.6465, -117.4387)
    private val point2Control1 = poseFactory.of(142.1887, 70.681, 0.0)
    private val point3 = poseFactory.of(24.1572, 24.0207, 153.4895)
    private val point3Control1 = poseFactory.of(71.2132, 0.309, 0.0)
    private val point4 = poseFactory.of(23.25, 117.3367, 63.3031)
    private val point4Control1 = poseFactory.of(-0.227, 71.1319, 0.0)

    fun path1(): Path = Paths.curve(start, path1Control1, path1Control2, path1).tangent()

    fun path2(): Path = Paths.curve(path1, point2Control1, point2).tangent()

    fun path3(): Path = Paths.curve(point2, point3Control1, point3).tangent()

    fun path4(): Path = Paths.curve(point3, point4Control1, point4).tangent()

    // Tell Ivy what order to follow the paths
    private fun autoRoutine(): Command {
        return sequential(
            follow(follower, path1()),
            follow(follower, path2()),
            follow(follower, path3()),
            follow(follower, path4())
        )
    }

    // Tie it into the auto
    override fun init() {
        // The init function and start function do not happen at the same time!
        // Start is when the Autonomous OpMode begins
        // Init is the first button you press BEFORE the start button
        Scheduler.reset()

        follower = Constants.create(hardwareMap)
        follower.setPose(start)
        follower.update()
    }

    override fun start() {
        schedule(autoRoutine())
    }

    override fun loop() {
        // This function runs continuously while the Autonomous is active
        follower.update()
        Scheduler.execute()

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}