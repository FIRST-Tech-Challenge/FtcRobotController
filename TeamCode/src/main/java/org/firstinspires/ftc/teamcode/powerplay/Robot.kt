package org.firstinspires.ftc.teamcode.powerplay

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.powerplay.subsystems.Intake
import org.firstinspires.ftc.teamcode.stateMachine.StateMachineBuilder
import org.firstinspires.ftc.teamcode.util.Subsystem

class Robot: Subsystem {
    val  intake = Intake()
    private val subsystems = mutableListOf(
        intake
    )

    enum class IntakeSequenceStates {
        START_INTAKE,
        STOP_INTAKE
    }

    val intakeSequence = StateMachineBuilder<IntakeSequenceStates>()
        .state(IntakeSequenceStates.START_INTAKE)
        .onEnter {
            startIntake()
        }
        .transition{
            isConeIn()
        }
        .state(IntakeSequenceStates.STOP_INTAKE)
        .onEnter{
            stopIntake()
        }
        .build()

    fun startIntake() {
        intake.turnOn()
    }

    fun stopIntake() {
        intake.turnOff()
    }

    fun reverseIntake() {
        intake.turnReverse()
    }

    fun isConeIn(): Boolean {
        return intake.isConeIn()
    }

    override fun init(hardwareMap: HardwareMap) {
        subsystems.forEach{it.init(hardwareMap)}
    }

    override fun update() {
        subsystems.forEach{it.update()}
    }

    override fun reset() {
        subsystems.forEach{it.reset()}
    }
}