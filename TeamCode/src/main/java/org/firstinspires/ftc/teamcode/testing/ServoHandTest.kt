package org.firstinspires.ftc.teamcode.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.ServoHand
import org.firstinspires.ftc.teamcode.util.GamepadState

@TeleOp(name="ServoHandTest")
class ServoHandTest : LinearOpMode() {
    override fun runOpMode() {
        val servo: Servo = hardwareMap.get(Servo::class.java, "s_one")
        val servoHand = ServoHand(servo, 0.0, 1.0)

        var pastGamepad1 = GamepadState()
        pastGamepad1.updateGamepadState(gamepad1)

        waitForStart()

        while (isStarted && !isStopRequested) {
            if (gamepad1.a && !pastGamepad1.a) {
                servoHand.toggle()
            }

            pastGamepad1.updateGamepadState(gamepad1)
        }
    }
}