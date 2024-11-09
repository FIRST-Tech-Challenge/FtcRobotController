package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.util.RobotLog
import dev.aether.collaborative_multitasking.MultitaskScheduler

@TeleOp
class ExampleAuto: LinearOpMode() {
    override fun runOpMode() {
        val schedule = MultitaskScheduler()
        val objective = schedule.task {
            startOnRequest()
            onStart { ->
                RobotLog.i("oh yeah we're back in business")
            }
            isCompleted { -> true }
        }
        waitForStart()
        while (opModeIsActive()) {
            schedule.tick()
            if (gamepad1.a) objective.requestStart()
            telemetry.addData(
                "Current task state",
                objective.state
            )
            schedule.displayStatus(true, true, telemetry::addLine)
            telemetry.update()
        }
    }
}