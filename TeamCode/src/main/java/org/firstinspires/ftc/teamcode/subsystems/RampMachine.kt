package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.POLLEN_EXTRACT
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.POLLEN_ONLY
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.POLLEN_UP

class RampMachine(private val hw: GerthaHardwareMap) {

    enum class State{
        BLOCK_NECTAR,
        FLOWER_INTAKE,
        UP
    }

    lateinit var state: State
    private var prevState = State.UP

    fun init() {
        hw.setPollenExtractor(POLLEN_UP)
        state = State.UP
    }

    fun update(gamepadState: GamepadState){

        val justTransitioned = (state != prevState)
        if (justTransitioned) {

        }
        prevState = state

        when (state){
            State.UP -> {
                if (justTransitioned){
                    hw.setPollenExtractor(POLLEN_UP)
                }
                if (gamepadState.rt1){
                    state = State.FLOWER_INTAKE
                }
            }
            State.FLOWER_INTAKE -> {
                if (justTransitioned){
                    hw.setPollenExtractor(POLLEN_EXTRACT)
                }
                if (gamepadState.rt1){
                    state = State.BLOCK_NECTAR
                }
            }
            State.BLOCK_NECTAR -> {
                if (justTransitioned){
                    hw.setPollenExtractor(POLLEN_ONLY)
                }
                if (gamepadState.rt1){
                    state = State.UP
                }
            }
        }
    }
}