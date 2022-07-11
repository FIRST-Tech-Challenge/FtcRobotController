package org.firstinspires.ftc.teamcode.components.servos

import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.initializableOnce

class Servos {
    var arm1: Servo by initializableOnce()
    var arm2: Servo by initializableOnce()
    var dep: Servo by initializableOnce()
    var fold: Servo by initializableOnce()
    var cap: Servo by initializableOnce()
}
