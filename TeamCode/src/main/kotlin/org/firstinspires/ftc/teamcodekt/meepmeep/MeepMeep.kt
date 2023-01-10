package org.firstinspires.ftc.teamcodekt.meepmeep

import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.meepmeep.MeepMeepPersistence
import ftc.rogue.blacksmith.units.GlobalUnits
import ftc.rogue.blacksmith.util.kt.pow
import org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.*

private val startPose = GlobalUnits.pos(91, -163, 90)

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

private fun mainTraj(drive: DriveShim) =
    Anvil.formTrajectory(drive, startPose)
        .waitInitialGoToDeposit()

        .awaitDeposit()

        .doTimes(5) {
            awaitGoToIntake(it)

            when (it) {
                4 -> awaitFastIntake()
                else -> awaitRegularIntake()
            }

            awaitGoToDeposit(it)

            awaitDeposit()
        }

        .build<TrajectorySequence>()

private fun Anvil.waitInitialGoToDeposit() = this
    .splineToSplineHeading(82.35, -11.35, 129.5, 117.5)

private fun Anvil.awaitGoToDeposit(it: Int) = this
    .splineToSplineHeading(82.35 + (it * .35), -11.35 - (it * 3.7), 131.5 + (it * 1.9), 155.0)

private fun Anvil.awaitGoToIntake(it: Int) =
    inReverse {
        splineTo(159.5 - ((it * .6) pow 1.25), -28.75 - (it * .85), 0.0)
    }

private fun Anvil.awaitDeposit() = this
    .waitTime(100)

private fun Anvil.awaitRegularIntake() = this
    .waitTime(225)

private fun Anvil.awaitFastIntake() = this
    .waitTime(20)
