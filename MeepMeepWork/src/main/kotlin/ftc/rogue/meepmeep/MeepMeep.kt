package ftc.rogue.meepmeep

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.noahbres.meepmeep.MeepMeep
import com.noahbres.meepmeep.MeepMeep.Background
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueDark
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder
import com.noahbres.meepmeep.roadrunner.DriveShim
import com.noahbres.meepmeep.roadrunner.trajectorysequence.TrajectorySequence
import ftc.rogue.blacksmith.Anvil
import ftc.rogue.blacksmith.meepmeep.MeepMeepPersistence
import ftc.rogue.blacksmith.util.toIn
import ftc.rogue.blacksmith.util.toRad

private val startPose = Pose2d(91.toIn(), (-163).toIn(), 90.toRad())

fun main() {
    val mm = MeepMeep(800)

    MeepMeepPersistence(mm).restore()

    val bot = DefaultBotBuilder(mm)
        .setColorScheme(ColorSchemeBlueDark())
        .setConstraints(40.0, 48.0, 320.8331314285714.toRad(), 320.8331314285714.toRad(), 4.89)
        .setDimensions(12.0, 12.0)
        .followTrajectorySequence(::traj)

    mm.setBackground(Background.FIELD_POWERPLAY_OFFICIAL)
        .setDarkMode(true)
        .setBackgroundAlpha(0.95f)
        .addEntity(bot)
        .start()
}

private fun traj(drive: DriveShim) =
    Anvil.formTrajectory(drive, startPose)
        .splineToSplineHeading(75.25, -17.0, 135.0, 117.5)

        .doTimes(5) {
            inReverse {
                splineTo(153.5, -29.5, 0.0)
            }

            splineToSplineHeading(75.25, -17.0, 135.0, 135.0)
        }

        .build<TrajectorySequence>()
