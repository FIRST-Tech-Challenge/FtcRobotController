package org.firstinspires.ftc.teamcode.util

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.drive.DriveConstants
import org.firstinspires.ftc.teamcode.util.math.Pose

object MeepMeep {
    @JvmStatic
    fun main(args: Array<String>) {
        val meepMeep = MeepMeep(600)
        val myBot = DefaultBotBuilder(meepMeep) // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setConstraints(60.0, 40.0, Math.toRadians(80.0), Math.toRadians(30.0), 13.25    )
            .setDimensions(12.5, 18.0)
            .followTrajectorySequence { drive: DriveShim ->
                drive.trajectorySequenceBuilder(Pose2d(6.25, 63.0, Math.toRadians(90.0)))
                    .setReversed(true)
                    //initial deposit
                    .lineToLinearHeading( Pose2d(-5.0, 47.0, -Math.toRadians(295.0)),
                        SampleMecanumDrive.getVelocityConstraint(35.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .setReversed(false)
                    //first half of warehouse one
                    .splineToSplineHeading( Pose2d(17.0, 66.0, Math.toRadians(360.0)), -Math.toRadians(360.0))
                    //second half of warehouse one
                    .splineToConstantHeading( Vector2d(42.0, 66.0), Math.toRadians(0.0),
                        SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20.0))
                    .splineToConstantHeading(Vector2d(46.0, 62.0, ), Math.toRadians(340.0))
                    .setReversed(true)
                    //exiting warehouse one
                    .splineToConstantHeading( Vector2d(10.0, 66.0), -Math.toRadians(180.0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    //deposit one
                    .lineToSplineHeading( Pose2d(-5.0, 44.2, Math.toRadians(85.0)),
                        SampleMecanumDrive.getVelocityConstraint(55.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .setReversed(false)
                    //first half of warehouse two
                    .splineToSplineHeading( Pose2d(17.0, 66.0, Math.toRadians(360.0)), Math.toRadians(360.0),
                        SampleMecanumDrive.getVelocityConstraint(45.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL)
                    )
                    //second half of warehouse two
                    .splineToConstantHeading( Vector2d(48.0, 66.0), Math.toRadians(0.0),
                        SampleMecanumDrive.getVelocityConstraint(40.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(20.0))
                    .setReversed(true)
                    //exiting warehouse two
                    .splineToConstantHeading( Vector2d(17.0, 66.0), -Math.toRadians(180.0),
                        SampleMecanumDrive.getVelocityConstraint(DriveConstants.MAX_VEL, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    //deposit two
                    .lineToConstantHeading(Vector2d(13.0, 64.0))
                    .splineTo( Vector2d(-5.0, 44.2), Math.toRadians(260.0),
                        SampleMecanumDrive.getVelocityConstraint(55.0, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH),
                        SampleMecanumDrive.getAccelerationConstraint(DriveConstants.MAX_ACCEL))
                    .setReversed(false)
                    .build()
            }


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start()
    }
}