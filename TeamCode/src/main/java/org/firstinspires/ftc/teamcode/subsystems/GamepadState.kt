package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.Gamepad

class GamepadState(private val gp1: Gamepad, private val gp2: Gamepad) {

    //for drive
    var jx1 = 0.0
    var jy1 = 0.0
    var rx1 = 0.0

    //outtake
    var lb1 = false
    var waslb1 = false

    //intake
    var rb1 = false
    var wasrb1 = false

    //cycle bar
    var rt1 = false
    var wasrt1 = false

    //shoot
    var b2 = false
    var wasb2 = false



    fun update() {
        jx1 = gp1.left_stick_x.toDouble()
        jy1 = -gp1.left_stick_y.toDouble()
        rx1 = gp1.right_stick_x.toDouble()

        lb1 = gp1.left_bumper && !waslb1
        waslb1 = gp1.left_bumper

        rb1 = gp1.right_bumper && !wasrb1
        wasrb1 = gp1.right_bumper

        rt1 = gp1.right_trigger > 0.5 && !wasrt1
        wasrt1 = gp1.right_trigger > 0.5

        b2 = gp2.b && ! wasb2
        wasb2 = gp2.b
    }
}