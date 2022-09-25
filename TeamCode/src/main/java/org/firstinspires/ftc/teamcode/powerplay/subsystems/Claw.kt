package org.firstinspires.ftc.teamcode.powerplay.subsystems

import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.util.Globals
import org.firstinspires.ftc.teamcode.util.Subsystem

class Claw: Subsystem {
    lateinit var claw: Servo

    private enum class ClawStates(val pos: Double) {
        OPEN(Globals.CLAW_OPEN),
        CLOSED(Globals.CLAW_CLOSED),
    }

    private var clawState = ClawStates.OPEN

    fun openClaw() {
        clawState = ClawStates.OPEN
    }

    fun closeClaw() {
        clawState = ClawStates.CLOSED
    }

    override fun init (hardwareMap: HardwareMap) {
        claw = hardwareMap.servo["Claw"]
    }

    override fun update() {
        claw.position = clawState.pos
    }

    override fun reset() {
        openClaw()
    }
}

