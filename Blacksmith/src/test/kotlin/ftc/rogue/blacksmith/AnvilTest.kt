package ftc.rogue.blacksmith

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import ftc.rogue.blacksmith.units.AngleUnit
import ftc.rogue.blacksmith.units.DistanceUnit
import ftc.rogue.blacksmith.units.TimeUnit
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import testutil.roadrunner.drive.SampleMecanumDrive
import testutil.roadrunner.trajectorysequence.TrajectorySequence
import kotlin.math.PI

// TODO: More thorough testing
internal class AnvilTest {
    private val drive = SampleMecanumDrive(null)

    @Test
    fun `roughly make sure Anvil's generated sequence is same as the actual builder`() {
        val expected = drive.trajectorySequenceBuilder(Pose2d())
            .forward(5.0)
            .turn(PI)
            .UNSTABLE_addTemporalMarkerOffset(1.0) {
                Thread.sleep(1000)
            }
            .waitSeconds(1.0)
            .setReversed(true)
            .splineTo(Vector2d(10.0, 10.0), PI)
            .setReversed(false)
            .forward(5.0)
            .turn(PI)
            .turn(PI)
            .forward(5.0)
            .turn(PI)
            .turn(PI)
            .forward(5.0)
            .turn(PI)
            .turn(PI)
            .build()

        Anvil.setUnits(DistanceUnit.INCHES, AngleUnit.RADIANS, TimeUnit.SECONDS)

        val actual = Anvil.formTrajectory(drive, Pose2d()) {
            forward(5.0)
            turn(PI)
            addTemporalMarker(1.0) {
                Thread.sleep(1000)
            }
            waitTime(1.0)
            inReverse {
                splineTo(10.0, 10.0, PI)
            }
            doTimes(3) {
                forward(5.0)
                doTimes(2) {
                    turn(PI)
                }
            }
        }.build<TrajectorySequence>()

        assertAll(
            { assertEquals(expected.duration(), actual.duration(), 1e-6) },
            { assertEquals(expected.size(), actual.size()) },
            { assertTrue(expected.end() epsilonEquals actual.end()) },
        )
    }
}
