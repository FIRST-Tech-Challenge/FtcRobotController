package org.firstinspires.ftc.teamcode.Team.OpModes.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.Team.ComplexRobots.Robot

/**
 *    Current does absolutely nothing
 */
@Autonomous(name = "Basic Test Autonomous")
class BasicAuto : LinearOpMode() {
    private val robot = Robot()
    override fun runOpMode() {
        robot.init(hardwareMap)
        waitForStart()

        //Add Built in functions here
    }
}