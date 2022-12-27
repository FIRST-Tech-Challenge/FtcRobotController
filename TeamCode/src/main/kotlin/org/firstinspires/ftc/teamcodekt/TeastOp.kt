package org.firstinspires.ftc.teamcodekt

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
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

        val mTelemetry = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        Scheduler.launchWhenReady(this) {
            mTelemetry.addData("FR", fr.currentPosition)
            mTelemetry.addData("FL", fl.currentPosition)
            mTelemetry.addData("BR", br.currentPosition)
            mTelemetry.addData("BL", bl.currentPosition)
            mTelemetry.update()
        }
    }
}
