package org.firstinspires.ftc.teamcodekt.meepmeep

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.meepmeep.MeepMeepPersistence
import ftc.rogue.blacksmith.units.GlobalUnits
import org.firstinspires.ftc.teamcode.AutoData
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.*
import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto

private val startPose = GlobalUnits.pos(-91, -163, 90)

fun main() {
    val mm = MeepMeep(800)

    MeepMeepPersistence(mm).restore()

    val bot = DefaultBotBuilder(mm)
        .setColorScheme(ColorSchemeBlueDark())
        .setConstraints(MAX_VEL, MAX_ACCEL, MAX_ANG_VEL, MAX_ANG_ACCEL, TRACK_WIDTH)
        .setDimensions(12.0, 12.0)
        .followTrajectorySequence(::mainTraj)

    mm.setBackground(Background.FIELD_POWERPLAY_OFFICIAL)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start()
}

//private fun mainTraj(drive: DriveShim) =
//    Anvil.formTrajectory(drive, startPose) {
//
//        splineToSplineHeading(-82.5, -12.75, 48.25, 65)
//
//    }.build<TrajectorySequence>()


private fun mainTraj(drive: DriveShim) =
    Anvil.formTrajectory(drive, startPose)

        .awaitInitialGoToDeposit()

        .awaitDeposit()

        .doTimes(RogueBaseAuto.NUM_CYCLES) {
            awaitGoToIntake(it)

            when (it) {
                RogueBaseAuto.LAST_CYCLE -> awaitFastIntake()
                else -> awaitRegularIntake()
            }

            awaitGoToDeposit(it)

            awaitDeposit()
        }

        .build<TrajectorySequence>()

private fun Anvil.awaitInitialGoToDeposit() = this
    .splineToSplineHeading(-82.5, -12.75, 48.25, 65)

private fun Anvil.awaitGoToDeposit(it: Int) = when (it) {
    0 -> splineToSplineHeading(-85.100, -10.250, 39.000, 25)
    1 -> splineToSplineHeading(-81.800, -14.905, 48.275, 25)
    2 -> splineToSplineHeading(-77.150, -16.450, 48.900, 25)
    3 -> splineToSplineHeading(-74.624, -19.575, 51.625, 25)
    4 -> splineToSplineHeading(-74.950, -17.950, 39.600, 25)
    else -> this
}

private fun Anvil.awaitGoToIntake(it: Int) = when (it) {
    0 -> splineTo(-161.0000, -25.850, 180)
    1 -> splineTo(-160.2375, -27.175, 180)
    2 -> splineTo(-158.9100, -28.400, 180)
    3 -> splineTo(-156.5975, -29.675, 180)
    4 -> splineTo(-154.6500, -30.950, 180)
    else -> noop
}.doInReverse()

private fun Anvil.awaitDeposit() = this
    .waitTime(100)

private fun Anvil.awaitRegularIntake() = this
    .waitTime(300)

private fun Anvil.awaitFastIntake() = this
    .waitTime(120)
