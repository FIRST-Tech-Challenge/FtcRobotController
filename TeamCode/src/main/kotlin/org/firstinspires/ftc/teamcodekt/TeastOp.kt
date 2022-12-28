package org.firstinspires.ftc.teamcodekt

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.Servo
import ftc.rouge.blacksmith.Scheduler
import ftc.rouge.blacksmith.util.kt.invoke
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcodekt.components.ClawColorSensor

@TeleOp
class TeastOp : LinearOpMode() {
    override fun runOpMode() {
//        val sensor = ClawColorSensor(hardwareMap)
        val port3 = hardwareMap<Servo>("3")
        val port4 = hardwareMap<Servo>("4")
        val port5 = hardwareMap<Servo>("5")

        val mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        Scheduler.launchWhenReady(this) {
//            mTelemetry.addData("teast", sensor.distanceSensor.getDistance(DistanceUnit.CM))
            if (gamepad1.a) {
                port3.position += 0.04
            }

            if (gamepad1.b) {
                port4.position += 0.04
            }

            if (gamepad1.x) {
                port5.position += 0.04
            }
            mTelemetry.update()
        }
    }
}
