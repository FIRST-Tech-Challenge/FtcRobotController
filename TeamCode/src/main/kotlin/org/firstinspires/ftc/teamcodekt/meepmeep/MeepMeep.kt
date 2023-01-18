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
    Anvil.formTrajectory(drive, startPose) {

        splineToSplineHeading(75.650, -18.750, 140.400, 155.0)

        waitTime(100)

        when (2) {
            1 -> {
                splineToLinearHeading(28, -32, 90, 128.375 + 90)
            }

            2 -> {
                splineTo(86, -33, 270).doInReverse()
            }

            3 -> {
                splineTo(154.65, -30.95, 0).doInReverse()
            }
            else -> this
        }

    }.build<TrajectorySequence>()
