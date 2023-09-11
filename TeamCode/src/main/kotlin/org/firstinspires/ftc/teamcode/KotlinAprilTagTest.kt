package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.lang.Thread.sleep

@TeleOp
class KotlinAprilTagTest: OpMode() {

    companion object { @JvmStatic private val USE_WEBCAM = true }

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    private lateinit var aprilTag: AprilTagProcessor;

    /**
     * The variable to store our instance of the vision portal.
     */
    private lateinit var visionPortal: VisionPortal;

    override fun init() { initAprilTagDetection() }

    fun initAprilTagDetection() {
        aprilTag = AprilTagProcessor.Builder().build(); // or AprilTagProcessor.easyCreateWithDefaults()

        // Create the vision portal the easy way.
        visionPortal = if (USE_WEBCAM) {
            VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName::class.java, "Webcam 1"),
                aprilTag
            )
        } else {
            VisionPortal.easyCreateWithDefaults(
                BuiltinCameraDirection.BACK,
                aprilTag
            )
        }
    }

    fun runAprilTagTelemetry() {
        val currentDetections = aprilTag.detections

        // Step through the list of detections and display info for each one.
        for (detection in currentDetections) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, if (detection.metadata != null) detection.metadata.name else "Unknown"))
            if (detection.metadata != null) {
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.")
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)")
        telemetry.addLine("RBE = Range, Bearing & Elevation")
    }

    override fun loop() {
        runAprilTagTelemetry()

        // Save CPU resources; can resume streaming when needed.
        if (gamepad1.dpad_down) {
            visionPortal.stopStreaming();
        } else if (gamepad1.dpad_up) {
            visionPortal.resumeStreaming();
        }

        // Share the CPU.
        try { sleep(20) } catch (e: InterruptedException) {  Thread.currentThread().interrupt()  }
    }

    override fun stop() {
        // Save more CPU resources when camera is no longer needed.
        visionPortal.close()
    }
}