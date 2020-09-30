package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.baseClasses.MecanumTeleOpMode
import org.firstinspires.ftc.teamcode.baseClasses.*

@TeleOp()
class MecaDriveTest: MecanumTeleOpMode() {
    private var lbPressed = false
    private var rbPressed = false
    private var ltPressed = false
    private var rtPressed = false

    override fun preMoveLoop() {
        if (lbPressed != gamepad1.left_bumper) {
            lbPressed = gamepad1.left_bumper
            hardware.base.leftFront.direction = direction(lbPressed)
        }

        if (rbPressed != gamepad1.right_bumper) {
            rbPressed = gamepad1.right_bumper
            hardware.base.rightFront.direction = direction(rbPressed)
        }

        if (ltPressed != gamepad1.left_trigger_pressed) {
            ltPressed = gamepad1.left_trigger_pressed
            hardware.base.leftBack.direction = direction(ltPressed)
        }

        if (rtPressed != gamepad1.right_trigger_pressed) {
            rtPressed = gamepad1.right_trigger_pressed
            hardware.base.rightBack.direction = direction(rtPressed)
        }

        logger { log ->
            log("Left Front  :: ${if (lbPressed) "REVERSE" else "FORWARD"}")
            log("Right Front :: ${if (rbPressed) "REVERSE" else "FORWARD"}")
            log("Left Back   :: ${if (ltPressed) "REVERSE" else "FORWARD"}")
            log("Right Back  :: ${if (rtPressed) "REVERSE" else "FORWARD"}")
        }
    }

    override fun mecaLoop() {
    }

    private fun direction(isReversed: Boolean): DcMotorSimple.Direction {
        return if (isReversed) DcMotorSimple.Direction.REVERSE else DcMotorSimple.Direction.FORWARD
    }
}