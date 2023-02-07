package org.firstinspires.ftc.teamcodekt.meepmeep

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.util.meepmeep.MeepMeepPersistence
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto
import org.firstinspires.ftc.teamcodekt.util.CycleException

private val startPose = GlobalUnits.pos(91, -163, 90)

fun main() {
    val mm = MeepMeep(800)

    MeepMeepPersistence(mm).restore()

    val bot = DefaultBotBuilder(mm)
        .setColorScheme(ColorSchemeBlueDark())
        .setConstraints(
            DriveConstants.MAX_VEL,
            DriveConstants.MAX_ACCEL,
            DriveConstants.MAX_ANG_VEL,
            DriveConstants.MAX_ANG_ACCEL,
            DriveConstants.TRACK_WIDTH
        )
        .setDimensions(12.0, 12.0)
        .followTrajectorySequence(::mainTraj)

    mm.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start()
}

private fun mainTraj(drive: DriveShim) =
    Anvil.forgeTrajectory(drive, startPose)
        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40.0, Math.toRadians(250.0), DriveConstants.TRACK_WIDTH))

        .forward(132)
        .turn(142.5)
        .goToDeposit(-1)
        .doTimes(RogueBaseAuto.NUM_CYCLES) {
            goToIntake(it)
            goToDeposit(it)
        }

        .build<TrajectorySequence>()

private fun Anvil.goToDeposit(it: Int) = when (it) {
    -1 -> lineToLinearHeading(83.2, -43.9, 180+39.5)
    0 -> splineTo(81, -43, 180+38)
    1 -> splineTo(81, -43, 180+36)
    2 -> splineTo(82, -43.1, 180+31)
    3 -> splineTo(82, -42, 180+30)
    4 -> splineTo(82, -41.5, 180+28)

    else -> throw CycleException()
}

private fun Anvil.goToIntake(it: Int) = when (it) {
    0 -> splineTo(166.3, -24.8, 0)
    1 -> splineTo(165.9, -24.7, 0)
    2 -> splineTo(164.2, -25, 0)
    3 -> splineTo(165.5, -24.1, 0)
    4 -> splineTo(164.3, -23.9, 0)
    else -> throw CycleException()
}.doInReverse()
