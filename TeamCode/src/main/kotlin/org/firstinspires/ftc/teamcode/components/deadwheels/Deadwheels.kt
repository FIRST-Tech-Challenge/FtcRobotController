package org.firstinspires.ftc.teamcode.components.deadwheels

import org.firstinspires.ftc.teamcode.util.initializableOnce

class Deadwheels {
    var left: Deadwheel by initializableOnce()
    var right: Deadwheel by initializableOnce()
    var back: Deadwheel by initializableOnce()
}