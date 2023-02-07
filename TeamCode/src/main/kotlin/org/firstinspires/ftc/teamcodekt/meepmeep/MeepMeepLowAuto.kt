package org.firstinspires.ftc.teamcodekt.meepmeep

import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.util.meepmeep.MeepMeepPersistence
import ftc.rogue.blacksmith.units.GlobalUnits
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

/**
 * LOW AUTO:
 * Run a trajectory to deposit on the low pole during auto
 * - Tiernan
 */
private fun mainTraj(drive: DriveShim) =
    Anvil.forgeTrajectory(drive, startPose)
        .awaitInitialGocToDeposit()
        .turn(46)
        .back(70)
        .strafeRight(4)
        .turn(-29)
        .forward(10)
        .back(16.5)

        .doTimes(4) {
            goToDeposit(it + 1)
            goToIntake(it + 1)
            waitTime(200)
        }

        .build<TrajectorySequence>()

private fun Anvil.awaitInitialGocToDeposit() = this
        .forward(132)
        .turn(-136)
        .forward(12.5)
        .waitTime(150)
        .back(12.5)

//  The offset values are from sin(32) and cos(32) degrees.
//  Used to spline in a straight line. This is advantageous to maintain localization better.
private fun Anvil.goToDeposit(it: Int) = when (it) {
    0 -> forward(21.5)
    1 -> forward(21.5)
    2 -> forward(21.5)
    3 -> forward(21.5)
    4 -> forward(21.5)
    else -> this
}

private fun Anvil.goToIntake(it: Int) = when (it) {
    0 -> back(21.5)
    1 -> back(21.5)
    2 -> back(21.5)
    3 -> back(21.5)
    4 -> back(21.5)
    else -> noop
}.doInReverse()
