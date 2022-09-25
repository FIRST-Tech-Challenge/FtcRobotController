package org.firstinspires.ftc.teamcode.powerplay.subsystems

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.Globals
import org.firstinspires.ftc.teamcode.util.Subsystem

class Slides: Subsystem {
    lateinit var slides: DcMotor

    private var slideState = SlideStates.OFF
    private enum class SlideStates(val power: Double) {
        UP(Globals.SLIDES_UP),
        DOWN(Globals.SLIDES_DOWN),
        OFF(Globals.SLIDES_OFF)
    }

    fun up() {
        slideState = SlideStates.UP
    }

    fun down() {
        slideState = SlideStates.DOWN
    }

    fun off() {
        slideState = SlideStates.OFF
    }

    override fun init(hardwareMap: HardwareMap) {
        slides = hardwareMap.dcMotor["Slides"]
    }

    override fun update() {
        slides.power = slideState.power
    }

    override fun reset() {
        off()
    }
}