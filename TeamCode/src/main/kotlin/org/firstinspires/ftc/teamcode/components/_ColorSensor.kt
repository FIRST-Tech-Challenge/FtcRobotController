package org.firstinspires.ftc.teamcode.components

import com.qualcomm.hardware.rev.RevColorSensorV3
import org.firstinspires.ftc.robotcore.external.Telemetry

fun RevColorSensorV3.logColorData(telemetry: Telemetry) {
    telemetry.addData("Red", red())
    telemetry.addData("Blue", blue())
    telemetry.addData("Green", green())
    telemetry.addData("Alpha", alpha())
}