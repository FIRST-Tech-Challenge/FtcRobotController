@file:Suppress("ClassName")

package ftc.rogue.blacksmith.proxies

import com.acmerobotics.roadrunner.geometry.Pose2d
import ftc.rogue.blacksmith.internal.proxies._SampleMecanumDrive
import io.mockk.*
import org.junit.jupiter.api.Assertions.*
import org.junit.jupiter.api.Test
import org.junit.jupiter.api.TestInstance
import testutil.roadrunner.drive.SampleMecanumDrive
import testutil.roadrunner.trajectorysequence.TrajectorySequence
import testutil.roadrunner.trajectorysequence.TrajectorySequenceBuilder

@TestInstance(TestInstance.Lifecycle.PER_CLASS)
internal class _SampleMecanumDriveTest {
    private val drive = mockk<SampleMecanumDrive>()
    private val _drive = _SampleMecanumDrive(drive)

    @Test
    fun `proxy correctly calls followTrajectorySequenceAsync`() {
        every { drive.followTrajectorySequenceAsync(any()) } just Runs

        val seq = TrajectorySequence()

        assertDoesNotThrow {
            _drive.followTrajectorySequenceAsync(seq)
        }
    }

    @Test
    fun `proxy correctly calls setPoseEstimate`() {
        every { drive.poseEstimate = any() } returns Unit

        assertDoesNotThrow {
            _drive.setPoseEstimate(Pose2d(1.0, 1.0))
        }
    }

    private fun newBuilder() = TrajectorySequenceBuilder(null, null, null, 0.0, 0.0)
}