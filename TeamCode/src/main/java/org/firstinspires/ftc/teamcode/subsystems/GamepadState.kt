package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import com.qualcomm.robotcore.hardware.Gamepad

class GamepadState(private val gp1: Gamepad, private val gp2: Gamepad) {

    //--------------------driver 1-----------------------------------
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

    //cycle pollen extractor
    var rt1 = false
    var wasrt1 = false

    //reset pinpoint heading to 0
    var start1 = false
    var wasStart1 = false

    //
    var lt1 = false
    var waslt1 = false

    //
    var a1 = false
    var wasa1 = false

    //
    var b1 = false
    var wasb1 = false

    //-----------------driver 2-----------------------

    //shoot
    var b2 = false
    var wasb2 = false

    //cycle turret
    var start2 = false
    var wasStart2 = false

    //corner reset pinpoint
    var back2 = false
    var wasBack2 = false

    //switch hives
    var a2 = false
    var wasA2 = false



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

        lt1 = gp1.left_trigger > 0.5 && !waslt1
        waslt1 = gp1.left_trigger > 0.5

        b1 = gp1.b && !wasb1
        wasb1 = gp1.b

        b2 = gp2.b && !wasb2
        wasb2 = gp2.b

        a1 = gp1.a && !wasa1
        wasa1 = gp1.a

        start1 = gp1.start && !wasStart1
        wasStart1 = gp1.start

        back2 = gp2.back && ! wasBack2
        wasBack2 = gp2.back

        start2 = gp2.start && !wasStart2
        wasStart2 = gp2.start

        a2 = gp2.a && !wasA2
        wasA2 = gp2.a
    }
}