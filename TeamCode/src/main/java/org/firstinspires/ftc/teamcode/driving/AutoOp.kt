@file:Suppress("unused", "UNUSED_VARIABLE")

package org.firstinspires.ftc.teamcode.driving

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.Hardware

@Autonomous(name="predefined movements", group = "test")
class AutoOp: LinearOpMode() {
    override fun runOpMode() {
        var hardware = Hardware(hardwareMap)

        val frontLeftMotor = Hardware.motorTL
        val backLeftMotor = Hardware.motorBL
        val backRightMotor = Hardware.motorBR
        val frontRightMotor = Hardware.motorTR
        waitForStart()

    }
}