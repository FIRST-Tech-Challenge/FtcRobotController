package org.firstinspires.ftc.teamcode.powerplay

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.powerplay.subsystems.Claw
import org.firstinspires.ftc.teamcode.powerplay.subsystems.Slides
import org.firstinspires.ftc.teamcode.util.Subsystem

class Robot: Subsystem {
    private val claw = Claw()
    private val slides = Slides()
    private val subsystems = mutableListOf(
        claw,slides
    )

    fun clawOpened() {
        claw.openClaw()
    }

    fun clawClosed() {
        claw.closeClaw()
    }

    fun slidesUp() {
        slides.up()
    }

    fun slidesDown() {
        slides.down()
    }

    fun slidesOff() {
        slides.off()
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