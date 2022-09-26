package org.firstinspires.ftc.teamcode.koawalib.commands.subsystem

import com.asiankoala.koawalib.command.group.SequentialGroup
import org.firstinspires.ftc.teamcode.koawalib.subsystems.SlideMove
import org.firstinspires.ftc.teamcode.koawalib.subsystems.Slides

class SlidesUpCmd (private val slides: Slides) : SequentialGroup(
    SlideMove(0.25, slides)
) {

    init {
        addRequirements(slides)
    }

    override fun end() {
        slides.setPower(0.0)
    }
}

class SlidesDownCmd (private val slides: Slides) : SequentialGroup(
    SlideMove(-0.25, slides)
) {

    init {
        addRequirements(slides)
    }

    override fun end() {
        slides.setPower(0.0)
    }
}