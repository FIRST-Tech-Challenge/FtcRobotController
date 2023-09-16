package org.firstinspires.ftc.teamcode.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.VisionProcessors

@Autonomous(name = "AprilTagTest1", group = "A")
class AprilTagTest1: DriveMethods() {
    override fun runOpMode() {
        initVision(VisionProcessors.APRILTAG)

        telemetry.addLine("Initialized AprilTag")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            addDataToTelemetry()
            telemetry.update()

            sleep(100)
        }
    }

}