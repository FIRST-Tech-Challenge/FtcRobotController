package org.firstinspires.ftc.teamcode.baseClasses

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix

class Logger {
    private var logs: MutableList<Pair<String, String>> = mutableListOf()
    private val telemetryPacket = TelemetryPacket()

    fun text(log: String) {
        val split = log.split("::")
        if (split.count() == 2) {
            logs.add(Pair(split[0].trim(), split[1].trim()))
            telemetryPacket.put(split[0].trim(), split[1].trim())
        } else {
            logs.add(Pair("", log))
            telemetryPacket.put("", log)
        }
    }

    fun text(title: String, value: String) {
        logs.add(Pair(title, value))
        telemetryPacket.put(title, value)
    }

    fun location(matrix: OpenGLMatrix) {
        val translation = matrix.translation
        val x = UnitDistance.mm(translation[0].toDouble()).inches
        val y = UnitDistance.mm(translation[1].toDouble()).inches

        val dotRadius = 10.0
        telemetryPacket.fieldOverlay()
                .setStroke("blue")
                .setFill("lightBlue")
                .setStrokeWidth(2)
                .fillCircle(x - dotRadius / 2, y - dotRadius / 2, dotRadius)
                .strokeCircle(x - dotRadius / 2, y - dotRadius / 2, dotRadius)
    }

    fun log(telemetry: Telemetry) {
        telemetry.clear()
        logs.forEach { telemetry.addData(it.first, it.second) }
        telemetry.update()
        FtcDashboard.getInstance().sendTelemetryPacket(telemetryPacket)
    }
}