package org.firstinspires.ftc.teamcode.Tests

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Components.CV.Pipelines.RFAprilCam
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer
import org.firstinspires.ftc.teamcode.Robots.BasicRobot
import org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController.Localizers.Tracker
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence

/**
 * Warren Zhou
 * 9/6/23
 * Odom localizer 8 24x24 squares...but Kotlin!
 */
@Autonomous(name = "KprilTagRRTest")

class KotlinAprilTagRRTest : LinearOpMode() {
    private lateinit var queuer : Queuer
    private lateinit var roadrun: SampleMecanumDrive
    override fun runOpMode() {
        val robot = BasicRobot(this, false)
        roadrun = SampleMecanumDrive(hardwareMap, Tracker.TrackType.ROADRUN_ODOMETRY)
        val startPose = Pose2d(40.0, 1.5 * 23.5, Math.toRadians(0.0))
        roadrun.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        val cam = RFAprilCam()
        roadrun.poseEstimate = startPose
        queuer = Queuer()
        var loops = 0
        waitForStart()
        if (isStopRequested) return
        val trajSeq2: TrajectorySequence =
            roadrun.trajectorySequenceBuilder(Pose2d(40.0, 1.5 * 23.5, 0.0))
                .splineTo(Vector2d(55.0, 1.5 * 23.5), 0.0)
                .setReversed(true)
                .lineTo(Vector2d(40.0, 1.5 * 23.5))
                .setReversed(false)
                .build()
        resetRuntime()
        BasicRobot.time = 0.0
        while (!isStopRequested && opModeIsActive()) {
//            for (i in 1..10)
//                followTrajAsync(trajSeq2)
            loops++
            BasicRobot.packet.put("loopTime", loops / BasicRobot.time)
            queuer.isFirstLoop = false
            robot.update()
            roadrun.update()
            cam.update()
        }
    }

    private fun followTrajAsync(traj: TrajectorySequence?) {
        if (queuer.queue(false, queuer.isStarted && !roadrun.isBusy)) {
            if (!roadrun.isBusy) roadrun.followTrajectorySequenceAsync(traj)
        }
    }
}