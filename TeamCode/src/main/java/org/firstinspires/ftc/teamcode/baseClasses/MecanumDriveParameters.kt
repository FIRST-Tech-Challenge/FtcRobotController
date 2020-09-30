package org.firstinspires.ftc.teamcode.baseClasses

/// Credits for the mecanum drive portion of this class go to FTC11848
/// https://www.reddit.com/r/FTC/comments/9ou0ib/mecanum_drive_code/e7wrp9f?utm_source=share&utm_medium=web2x&context=3

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin

class MecanumDriveParameters private constructor(
        val driveStickY: Double,
        val driveStickX: Double,
        val turnStickX: Double,
        val forwardAmount: Double,
        val strafeAmount: Double,
) {
    companion object {
        @JvmStatic fun fromGamepad(gamepad: Gamepad, hardware: RobotHardware): MecanumDriveParameters {
            val driveX = gamepad.left_stick_y.toDouble()
            val driveY = gamepad.left_stick_x.toDouble()
            val turnX = gamepad.right_stick_x.toDouble()
            return fromXYTurn(x = driveX, y = driveY, turn = turnX, hardware = hardware)
        }


        /**
         * Create the mecanum parameters required to drive in the specified direction at the specified power
         *
         * @param direction the heading to drive - clockwise where 0° corresponds to moving the stick up on the controller
         * @param magnitude the power to apply - as a double in the range 0 ≤ `magnitude` ≤ 1
         */
        @JvmStatic fun fromDirectionAndMagnitude(direction: UnitAngle, magnitude: Double, hardware: RobotHardware): MecanumDriveParameters {
            val power = Range.clip(magnitude, 0.0, 1.0)

            val degrees = direction.degrees
            val anticlockwise = (-degrees + 360) % 360
            val fromPositiveX = (anticlockwise + 90) % 360
            val radians = UnitAngle.degrees(fromPositiveX).radians

            var dx = abs(power * cos(radians))
            var dy = abs(power * sin(radians))

            when (direction.quadrant) {
                UnitAngle.Quadrant.I -> dy = -dy
                UnitAngle.Quadrant.II -> { dx = -dx; dy = -dy }
                UnitAngle.Quadrant.III -> dx = -dx
                else -> {}
            }

            return fromXYTurn(x = dx, y = dy, turn = 0.0, hardware = hardware)
        }


        @JvmStatic private fun fromXYTurn(x: Double, y: Double, turn: Double, hardware: RobotHardware): MecanumDriveParameters {
            val orientation: Orientation = hardware.getOrientation()
            val angle = UnitAngle.degrees(orientation.firstAngle.toDouble())

            val forward = y * cos(angle.radians) + x * sin(angle.radians)
            val strafe = -y * sin(angle.radians) + x * cos(angle.radians)

            return MecanumDriveParameters(
                    driveStickX = x,
                    driveStickY = y,
                    turnStickX = turn,
                    forwardAmount = forward,
                    strafeAmount = strafe
            )
        }
    }
}
