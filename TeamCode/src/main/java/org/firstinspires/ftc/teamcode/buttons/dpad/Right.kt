package org.firstinspires.ftc.teamcode.buttons.dpad

import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.imgproc.ImgProc

class Right(telemetry: Telemetry) {

    var img: ImgProc = ImgProc()

    init {
        this.img.start(telemetry)
    }
}