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

private val startPose = GlobalUnits.pos(-84.5, -163, -90)

fun main() {
    val mm = MeepMeep(800, 1000)

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
        .inReverse {
            splineTo(-84.5, -36.5, 90)

            turn(44)
        }
        
        .splineTo(-73.2, -47.8, -44)
        .awaitDeposit()

        .inReverse {
            splineTo(-156, -31, 180)
            awaitIntake1()
        }

        .forward(10)
        .turn(-28.5)
        .strafeRight(12.5)
        .forward(18)
        .awaitDeposit()

        .doTimes(4) {
            back(25)
            awaitIntake2()
            forward(25)
            awaitDeposit()
        }

        .build<TrajectorySequence>()

private fun Anvil.awaitDeposit() = this
    .waitTime(350)

// Regular intaking w/ claw open wide
private fun Anvil.awaitIntake1() = this
    .waitTime(350)

// Intaking w/ claw narrow & intake wheels spinning
private fun Anvil.awaitIntake2() = this
    .waitTime(350)
