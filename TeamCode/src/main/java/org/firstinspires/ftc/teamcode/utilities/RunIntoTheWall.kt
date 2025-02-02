package org.firstinspires.ftc.teamcode.utilities

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import dev.aether.collaborative_multitasking.ITask
import dev.aether.collaborative_multitasking.MultitaskScheduler
import dev.aether.collaborative_multitasking.OneShot
import dev.aether.collaborative_multitasking.Scheduler
import dev.aether.collaborative_multitasking.TaskGroup
import dev.aether.collaborative_multitasking.ext.Pause
import org.firstinspires.ftc.teamcode.Hardware

@TeleOp
class RunIntoTheWall : LinearOpMode() {
    lateinit var scheduler: MultitaskScheduler
    lateinit var hardware: Hardware

    private fun run(it: () -> Unit): ITask {
        return OneShot(scheduler, it)
    }

    private fun wait(seconds: Number): ITask {
        return Pause(scheduler, seconds.toDouble())
    }

    private fun await(milliseconds: Number): ITask {
        return wait(milliseconds.toDouble() / 1000)
    }

    private fun groupOf(contents: (Scheduler) -> Unit): TaskGroup {
        return TaskGroup(scheduler).with(contents)
    }

    private fun hardwareInit() {
        hardware.backLeft.mode = RUN_WITHOUT_ENCODER
        hardware.frontLeft.mode = RUN_WITHOUT_ENCODER
        hardware.backRight.mode = RUN_WITHOUT_ENCODER
        hardware.frontRight.mode = RUN_WITHOUT_ENCODER
        hardware.clawFlip.position = Hardware.FLIP_UP
        hardware.clawFront.position = Hardware.FRONT_OPEN
        hardware.clawTwist.position = Hardware.CLAW_TWIST_INIT

        hardware.arm.targetPosition = 0
        hardware.arm.mode = RUN_TO_POSITION
        hardware.arm.power = 0.3
        hardware.wrist.position = 0.28
        hardware.claw.position = Hardware.CLAW_CLOSE

        // we don't have the proxy object to handle this for us
        // so manually implement the inversion
        hardware.horizontalSlide.position = Hardware.RIGHT_SLIDE_IN
        hardware.horizontalLeft.position = 1.05 - Hardware.RIGHT_SLIDE_IN

        hardware.lightLeft.position = Hardware.LAMP_PURPLE
        hardware.lightRight.position = Hardware.LAMP_PURPLE
    }

    fun doIt(): TaskGroup = groupOf {
        it.add(run {
            hardware.frontLeft.power = 0.6
            hardware.frontRight.power = -0.6
            hardware.backLeft.power = -0.6
            hardware.backRight.power = 0.6
        }).then(await(800)).then(run {
            hardware.frontLeft.power = -0.3
            hardware.frontRight.power = 0.3
            hardware.backLeft.power = 0.3
            hardware.backRight.power = -0.3
        }).then(await(300))
    }

    override fun runOpMode() {
        scheduler = MultitaskScheduler()
        hardware = Hardware(hardwareMap)
        hardwareInit()

        var chainWith = scheduler.add(doIt())
        for (i in 0..<20) chainWith = chainWith.then(doIt())
        chainWith.then(run{hardware.driveMotors.setAll(0.0)})

        while (opModeIsActive()) {
            scheduler.tick()
            scheduler.displayStatus(true, true, telemetry::addLine)
            telemetry.update()
        }
    }
}