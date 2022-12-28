package org.firstinspires.ftc.teamcodekt.components

import com.qualcomm.hardware.rev.RevColorSensorV3
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import ftc.rouge.blacksmith.util.kt.invoke

class ClawColorSensor(hwMap: HardwareMap) {
    val colorSensor = hwMap<ColorSensor>(DeviceNames.COLOR_SENSOR)
    val distanceSensor = hwMap<DistanceSensor>(DeviceNames.COLOR_SENSOR)

//    val s = hwMap<RevColorSensorV3>(DeviceNames.COLOR_SENSOR)
}
