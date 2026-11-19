package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.hardware

import org.firstinspires.ftc.teamcode.hardware.RobotPose
import kotlin.math.PI
import kotlin.unaryMinus

class PoseSet(val red: Boolean) {

    private val RobotPose.bind: RobotPose
        get() = if (!red) RobotPose(
            -this.x,
            -this.y,
            this.a + PI,
            this.turret?.let { -it }
        ) else this

    companion object {

        private val CORNER_START = RobotPose(62.75, 62.75, PI)
        private val OTHER_FLOWER_PRESCORE = RobotPose(51.3, 20.4, 0.0)
        private val OTHER_FLOWER_INTAKE = RobotPose(56.7, 20.4, 0.0)
        private val SCORE_LEFT = RobotPose(57.3, 38.4, PI / 6.0)
        private val SCORE_RIGHT = RobotPose(-57.3, 38.4, 5.0 * PI /6.0)
        private val DRIVER_FLOWER_PRESCORE = RobotPose(-21.4, 54.3, PI/2)
        private val DRIVER_FLOWER_INTAKE = RobotPose(-21.4, 59.7, PI/2)
        private val PARK_RIGHT = RobotPose(22.7,64.1,PI/2)
        private val PARK_LEFT = RobotPose(49.3, 63.6, PI/2)
        private val PREGARDEN = RobotPose(-53.3, 65.5, PI)
        private val GARDEN = RobotPose(-62.0, 65.5, PI)
        private val HIVE_WAYPOINT_LEFT = RobotPose(48.0, 45.0, PI / 6)
        private val HIVE_WAYPOINT_RIGHT = RobotPose(-48.0, 45.0, 5 * PI / 6.0)

        private val RIGHT_HIVE_AT = RobotPose(-20.0, 12.0, 0.0)
        private val LEFT_HIVE_AT = RobotPose(20.0, 12.0, 0.0)

        @JvmField
        val RED = PoseSet(false)
        @JvmField
        val BLUE = PoseSet(true)
    }

    @JvmField
    val CORNER_START = Companion.CORNER_START.bind
    @JvmField
    val OTHER_FLOWER_PRESCORE = Companion.OTHER_FLOWER_PRESCORE.bind
    @JvmField
    val OTHER_FLOWER_INTAKE = Companion.OTHER_FLOWER_INTAKE.bind
    @JvmField
    val SCORE_LEFT = Companion.SCORE_LEFT.bind
    @JvmField
    val SCORE_RIGHT = Companion.SCORE_RIGHT.bind
    @JvmField
    val DRIVER_FLOWER_PRESCORE = Companion.DRIVER_FLOWER_PRESCORE.bind
    @JvmField
    val DRIVER_FLOWER_INTAKE = Companion.DRIVER_FLOWER_INTAKE.bind
    @JvmField
    val PARK_RIGHT = Companion.PARK_RIGHT.bind
    @JvmField
    val PARK_LEFT = Companion.PARK_LEFT.bind
    @JvmField
    val PREGARDEN = Companion.PREGARDEN.bind
    @JvmField
    val GARDEN = Companion.GARDEN.bind
    @JvmField
    val HIVE_WAYPOINT_LEFT = Companion.HIVE_WAYPOINT_LEFT.bind
    @JvmField
    val HIVE_WAYPOINT_RIGHT = Companion.HIVE_WAYPOINT_RIGHT.bind
    @JvmField
    val RIGHT_HIVE_AT = Companion.RIGHT_HIVE_AT.bind
    @JvmField
    val LEFT_HIVE_AT = Companion.LEFT_HIVE_AT.bind

    fun getLEFT_HIVE_AT(): RobotPose {
        return LEFT_HIVE_AT
    }

    fun getRIGHT_HIVE_AT(): RobotPose {
        return RIGHT_HIVE_AT
    }
}