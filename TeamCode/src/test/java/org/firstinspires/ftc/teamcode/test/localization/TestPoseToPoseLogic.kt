package org.firstinspires.ftc.teamcode.test.localization

import org.firstinspires.ftc.teamcode.mmooover.Motion
import org.firstinspires.ftc.teamcode.mmooover.Pose
import org.firstinspires.ftc.teamcode.mmooover.PoseFromToProcessor
import org.firstinspires.ftc.teamcode.test.mocks.DummyRobot
import org.junit.jupiter.api.Test
import java.lang.Math.toRadians
import kotlin.math.abs

class TestPoseToPoseLogic {
    companion object {
        const val EPSILON = 1e-6
    }

    val realRobotNotClickbait = DummyRobot()

    private fun runPFTP(from: Pose, to: Pose): Motion {
        val p2p = PoseFromToProcessor(from)
        p2p.update(0.0, from)
        return p2p.getMotionToTarget(to, realRobotNotClickbait)
    }

    private val Number.deg get() = toRadians(this.toDouble())

    fun p(x: Number, y: Number, r: Number) = Pose(x.toDouble(), y.toDouble(), r.toDouble())

    internal class AssertHelper(val forward: Double, val right: Double, val turn: Double) {
        private val problems: MutableList<String> = mutableListOf()
        inline fun a(condition: Boolean, message: () -> String) {
            if (!condition) problems.add(message())
        }
        inline fun approxZero(item: Double, message: () -> String) = a(abs(item) <= EPSILON, message)

        internal fun finalize() {
            if (problems.isEmpty()) return
            val withLabels = problems.size > 1
            throw AssertionError(buildString {
                if (withLabels) append("${problems.size} failures:\n")
                for ((i, problem) in problems.withIndex()) {
                    if (i > 0) append("\n")
                    if (withLabels) append("${i + 1}: ")
                    append(problem)
                }
            })
        }
    }
    internal fun validate(motion: Motion, configure: AssertHelper.() -> Unit) {
        val target = AssertHelper(motion.forward, motion.right, motion.turn)
        target.configure()
        target.finalize()
    }

    @Test fun `straight on`() {
        val result = runPFTP(
            p(0, 0, 0),
            p(24, 0, 0)
        )
        validate(result) {
            a(forward >= 0.1) { "Doesn't move forward (actually $forward)" }
            approxZero(right) { "Shouldn't strafe (actually $right)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `straight backwards`() {
        val result = runPFTP(
            p(0, 0, 0),
            p(-24, 0, 0)
        )
        validate(result) {
            a(forward <= -0.1) { "Doesn't move backward (actually $forward)" }
            approxZero(right) { "Shouldn't strafe (actually $right)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `strafe left +y`() {
        val result = runPFTP(
            p(0, 0, 0),
            p(0, 24, 0)
        )
        validate(result) {
            approxZero(forward) { "Shouldn't move forward (actually $forward)" }
            a(right <= -0.1) { "Doesn't strafe left (actually $right, should be negative)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `strafe right -y`() {
        val result = runPFTP(
            p(0, 0, 0),
            p(0, -24, 0)
        )
        validate(result) {
            approxZero(forward) { "Shouldn't move forward (actually $forward)" }
            a(right >= 0.1) { "Doesn't strafe right (actually $right, should be positive)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `straight on +y, rotated 90deg`() {
        val result = runPFTP(
            p(0, 0, 90.deg),
            p(0, 24, 90.deg)
        )
        validate(result) {
            a(forward >= 0.1) { "Doesn't move forward (actually $forward)" }
            approxZero(right) { "Shouldn't strafe (actually $right)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `straight backwards -y, rotated 90deg`() {
        val result = runPFTP(
            p(0, 0, 90.deg),
            p(0, -24, 90.deg)
        )
        validate(result) {
            a(forward <= -0.1) { "Doesn't move backward (actually $forward)" }
            approxZero(right) { "Shouldn't strafe (actually $right)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `strafe right +x, rotated 90deg`() {
        val result = runPFTP(
            p(0, 0, 90.deg),
            p(24, 0, 90.deg)
        )
        validate(result) {
            approxZero(forward) { "Shouldn't move forward (actually $forward)" }
            a(right >= 0.1) { "Doesn't strafe right (actually $right, should be positive)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `strafe left -x, rotated 90deg`() {

        val result = runPFTP(
            p(0, 0, 90.deg),
            p(-24, 0, 90.deg)
        )
        validate(result) {
            approxZero(forward) { "Shouldn't move forward (actually $forward)" }
            a(right <= -0.1) { "Doesn't strafe left (actually $right, should be negative)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `straight on -x, rotated 180deg`() {
        val result = runPFTP(
            p(0, 0, 180.deg),
            p(-24, 0, 180.deg)
        )
        validate(result) {
            a(forward >= 0.1) { "Doesn't move forward (actually $forward)" }
            approxZero(right) { "Shouldn't strafe (actually $right)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `straight backwards +x, rotated 180deg`() {
        val result = runPFTP(
            p(0, 0, 180.deg),
            p(24, 0, 180.deg)
        )
        validate(result) {
            a(forward <= -0.1) { "Doesn't move backward (actually $forward)" }
            approxZero(right) { "Shouldn't strafe (actually $right)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `strafe right +y, rotated 180deg`() {
        val result = runPFTP(
            p(0, 0, 180.deg),
            p(0, 24, 180.deg)
        )
        validate(result) {
            approxZero(forward) { "Shouldn't move forward (actually $forward)" }
            a(right >= 0.1) { "Doesn't strafe right (actually $right, should be positive)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }

    @Test fun `strafe left -y, rotated 180deg`() {

        val result = runPFTP(
            p(0, 0, 180.deg),
            p(0, -24, 180.deg)
        )
        validate(result) {
            approxZero(forward) { "Shouldn't move forward (actually $forward)" }
            a(right <= -0.1) { "Doesn't strafe left (actually $right, should be negative)" }
            approxZero(turn) { "Shouldn't turn (actually $turn)" }
        }
    }
}