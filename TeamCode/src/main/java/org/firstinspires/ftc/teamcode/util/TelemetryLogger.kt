package org.firstinspires.ftc.teamcode.util

import org.firstinspires.ftc.robotcore.external.Telemetry

enum class TelemetryType {
    DEBUG, INFO, WARNING, ERROR
}

class TelemetryLogger(val telemetry: Telemetry) {
    private fun log(message: String, type: TelemetryType = TelemetryType.INFO) {
        when (type) {
            TelemetryType.DEBUG -> telemetry.addData("[DEBUG]", message)
            TelemetryType.INFO -> telemetry.addData("[INFO]", message)
            TelemetryType.WARNING -> telemetry.addData("[WARNING]", message)
            TelemetryType.ERROR -> telemetry.addData("[ERROR]", message)
        }

//        telemetry.update()
    }

    fun debug(message: String) = log(message, TelemetryType.DEBUG)
    fun info(message: String) = log(message, TelemetryType.INFO)
}