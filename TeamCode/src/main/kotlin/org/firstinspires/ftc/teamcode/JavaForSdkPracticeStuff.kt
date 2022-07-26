@file:Suppress("MemberVisibilityCanBePrivate", "ControlFlowWithEmptyBody")

package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit as DU
import org.firstinspires.ftc.teamcode.util._get
import java.util.function.ToIntFunction
import kotlin.properties.Delegates

lateinit var hwMap: HardwareMap

var DcMotorSimple._power: Number
    get() = throw IllegalStateException("Use the 'power' property to read instead")
    set(value) {
        power = value.toDouble()
    }

object ProgrammingBoard1 {
    val colorSensor: ColorSensor = hwMap._get("sensor_color_distance")
    fun getAmountBlue() = colorSensor.blue()
}

object ProgrammingBoard2 {
    val distanceSensor: DistanceSensor = hwMap._get("sensor_color_distance")
    val motor = hwMap._get<DcMotorEx>("motor")
    tailrec fun imaginaryLoop() {
        motor._power = if (distanceSensor.getDistance(DU.CM) < 10) 0 else .5
        imaginaryLoop()
    }
}

fun getRuntime() = 1.0
fun resetStartTime() {}
object ProgrammingBoard3 {
    val motor: DcMotorEx = hwMap._get("motor")
    val touchSensor: DigitalChannel = hwMap._get("touch_sensor")

    var lastStateChangeTime by Delegates.notNull<Double>()

    fun imaginaryStart() {
        resetStartTime()
        imaginaryLoop()
    }

    tailrec fun imaginaryLoop() {
        if (motor.power < 1) {
            motor.power = getRuntime().coerceIn(.25, 1.0)
        }

        if (motor.power == 1.0 && !touchSensor.state) {
            motor.power = 0.0
        }

        imaginaryLoop()
    }
}

object ProgrammingBoard4 {
    val motor: DcMotorEx = hwMap._get("motor")
    val distanceSensor: DistanceSensor = hwMap._get("sensor_color_distance")
    val servo: Servo = hwMap._get("servo")

    fun imaginaryLoop() {
        if (distanceSensor.getDistance(DU.CM) > 10 && getRuntime() < 5) {
            motor.power = 1.0
        } else {
            motor.power = 0.0
            servo.position = 0.5
        }
    }
}

object ProgrammingBoard5 {
    val motor: DcMotorEx = hwMap._get("motor")
    val touchSensor: DigitalChannel = hwMap._get("touch_sensor")
    var touchedBefore = false
    tailrec fun imaginaryLoop() {
        if (!touchSensor.state && !touchedBefore) {
//            gamepad1.rumble(100)
        }

        imaginaryLoop()
    }
}
