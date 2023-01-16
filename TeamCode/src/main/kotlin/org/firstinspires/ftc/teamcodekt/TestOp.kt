package org.firstinspires.ftc.teamcodekt

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.Scheduler
import ftc.rogue.blacksmith.listeners.ReforgedGamepad
import ftc.rogue.blacksmith.util.kt.invoke
import org.firstinspires.ftc.teamcodekt.components.Camera
import org.firstinspires.ftc.teamcodekt.opmodes.auto.RogueBaseAuto

@TeleOp
class TestOp1 : BlackOp() {
    override fun go() {
        val camera = Camera()

        val driver = ReforgedGamepad(gamepad1)

        camera.update()

        camera.waitForStartWithVision(this)

//        Scheduler.launchOnStart(this) {
//            camera.update()
//        }
    }
}

@TeleOp
@Config
class TestOp2 : RogueBaseAuto() {
    companion object {
        @JvmField
        var TEST_LIFT_HEIGHT = 100
    }

    override fun execute() {
        val parallelEncoder = hwMap<DcMotorEx>( "FL")
        val perpendicularEncoder =  hwMap<DcMotorEx>( "BR")

        Scheduler.launchOnStart(this) {
            mTelemetry.addData("Parallel", parallelEncoder.currentPosition)
            mTelemetry.addData("Perpendicular", perpendicularEncoder.currentPosition)
            mTelemetry.update()
        }
    }
}
