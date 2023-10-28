package org.firstinspires.ftc.teamcode.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables

@Autonomous(name="Just Drive Left, we need an Auto")
class JustDriveDuckingLeft: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()
        waitForStart()

        val speed: Double = 0.5

        Variables.motorBL!!.power = -speed
        Variables.motorBR!!.power = -speed
        Variables.motorFL!!.power = speed
        Variables.motorFR!!.power = speed
        sleep(3000)
        Variables.motorBL!!.power = 0.0
        Variables.motorBR!!.power = 0.0
        Variables.motorFL!!.power = 0.0
        Variables.motorFR!!.power = 0.0
    }
}