package org.firstinspires.ftc.teamcode.powerplay.subsystems

import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.util.Globals
import org.firstinspires.ftc.teamcode.util.Subsystem

class Intake: Subsystem {
    lateinit var intake: DcMotor
    lateinit var dSensor: Rev2mDistanceSensor

    private enum class IntakeStates(val power: Double) {
        ON(Globals.INTAKE_ON),
        OFF(Globals.INTAKE_OFF),
        REVERSE(Globals.INTAKE_REVERSE),
    }

    private enum class SensorStates {
        NONE,
        CONE_IN
    }

    private val sensorThreshold = 40.0
    private var sensorRead = 0.0
    private var intakeState = IntakeStates.OFF
    private var sensorState = SensorStates.NONE

    fun turnOn() {
        intakeState = IntakeStates.ON
    }

    fun turnOff() {
        intakeState = IntakeStates.OFF
    }

    fun turnReverse() {
        intakeState = IntakeStates.REVERSE
    }

    fun isConeIn(): Boolean {
        return sensorState == SensorStates.CONE_IN
    }

    override fun init (hardwareMap: HardwareMap) {
        intake = hardwareMap.dcMotor["Intake"]
        dSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "distanceSensor") as Rev2mDistanceSensor
    }

    override fun update() {
        intake.power = intakeState.power

        sensorRead = dSensor.getDistance(DistanceUnit.MM)
        sensorState = when {
            sensorRead < sensorThreshold -> SensorStates.CONE_IN
            else -> SensorStates.NONE
        }
    }

    override fun updateTelemetry() {
        telemetry.addData("intake power", intake.power)
        telemetry.addData("dSensor read", sensorRead)
        telemetry.addData("dSensor state", sensorState)
        telemetry.update()
    }

    override fun reset() {
        turnOff()
    }
}
