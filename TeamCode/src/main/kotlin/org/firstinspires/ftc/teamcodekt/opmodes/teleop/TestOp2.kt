package org.firstinspires.ftc.teamcodekt.opmodes.teleop

import com.arcrobotics.ftclib.hardware.SimpleServo
import com.arcrobotics.ftclib.kotlin.extensions.util.clamp
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import ftc.rouge.blacksmith.Scheduler
import ftc.rouge.blacksmith.listeners.ReforgedGamepad
import org.firstinspires.ftc.teamcodekt.components.DeviceNames

@TeleOp
class TuningOp : LinearOpMode() {
    lateinit var arm: SimpleServo

    override fun runOpMode() {
        arm = SimpleServo(hardwareMap, DeviceNames.CLAW_SERVO, 0.0, 180.0)

        val driver = ReforgedGamepad(gamepad1)

        driver.right_trigger.onRise {
            arm.position += .05
        }

        driver.left_trigger.onRise {
            arm.position -= .05
        }

        waitForStart()
        
        Scheduler.launch(this)
    }
}
