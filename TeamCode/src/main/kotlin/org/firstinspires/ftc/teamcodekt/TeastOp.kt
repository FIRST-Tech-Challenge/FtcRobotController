package org.firstinspires.ftc.teamcodekt

import com.arcrobotics.ftclib.hardware.motors.Motor
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import ftc.rouge.blacksmith.Scheduler

@TeleOp
class TeastOp : LinearOpMode() {
    override fun runOpMode() {
        val fr = Motor(hardwareMap, "FR") // Lift (max 1620) (high 1600) (mid 1170) (low 737)
        val fl = Motor(hardwareMap, "FL") // Left Deadwheel
        val br = Motor(hardwareMap, "BR") // Right Deadhweel
        val bl = Motor(hardwareMap, "BL") // Horizontal deadwheel

        fr.resetEncoder()
        fl.resetEncoder()
        br.resetEncoder()
        bl.resetEncoder()

        Scheduler.launchWhenReady(this) {
            telemetry.addData("FR", fr.currentPosition)
            telemetry.addData("FL", fl.currentPosition)
            telemetry.addData("BR", br.currentPosition)
            telemetry.addData("BL", bl.currentPosition)
            telemetry.update()
        }
    }
}
