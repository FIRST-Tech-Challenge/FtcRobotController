package org.firstinspires.ftc.teamcodekt

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DistanceSensor
import ftc.rouge.blacksmith.Scheduler
import ftc.rouge.blacksmith.util.kt.invoke
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

@TeleOp
class TeastOp : LinearOpMode() {
    override fun runOpMode() {
        val ds = hardwareMap<DistanceSensor>("CS")

        val mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        Scheduler.launchWhenReady(this) {
            mTelemetry.addData("teast", ds.getDistance(DistanceUnit.CM))
            mTelemetry.update()
        }
    }
}
