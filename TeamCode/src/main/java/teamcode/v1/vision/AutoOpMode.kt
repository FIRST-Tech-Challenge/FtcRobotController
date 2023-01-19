package teamcode.v1.vision

import com.asiankoala.koawalib.command.KOpMode
import com.asiankoala.koawalib.logger.Logger
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.koawalib.vision.SleevePipeline
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

open class AutoOpMode : KOpMode(photonEnabled = true) {
    lateinit var camera : OpenCvCamera
    lateinit var pipeline : SleevePipeline

    var fx = 578.272
    var fy = 578.272
    var cx = 402.145
    var cy = 221.506

    var tagsize = 0.166

    var LEFT = 1
    var MIDDLE = 2
    var RIGHT = 3
    var tagOfInterest: AprilTagDetection? = null

    override fun mInit() {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            hardwareMap.get(
                WebcamName::class.java, "Webcam"
            ), cameraMonitorViewId
        )
        pipeline = SleevePipeline(tagsize, fx, fy, cx, cy)
        camera.setPipeline(pipeline)
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {}
        })
    }

    override fun mInitLoop() {
        val currentDetections: ArrayList<AprilTagDetection> =
            pipeline.latestDetections
        if (currentDetections.size != 0) {
            var tagFound = false
            for (tag in currentDetections) {
                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag
                    tagFound = true
                    break
                }
            }
            if (tagFound) {
                Logger.addTelemetryLine("Tag of interest is in sight!\n\nLocation data:")
                tagToTelemetry(tagOfInterest)
            } else {
                Logger.addTelemetryLine("Don't see tag of interest :(")
                if (tagOfInterest == null) {
                    Logger.addTelemetryLine("(The tag has never been seen)")
                } else {
                    Logger.addTelemetryLine("\nBut we HAVE seen the tag before; last seen at:")
                    tagToTelemetry(tagOfInterest)
                }
            }
        } else {
            Logger.addTelemetryLine("Don't see tag of interest :(")
            if (tagOfInterest == null) {
                Logger.addTelemetryLine("(The tag has never been seen)")
            } else {
                Logger.addTelemetryLine("\nBut we HAVE seen the tag before; last seen at:")
                tagToTelemetry(tagOfInterest)
            }
        }
    }

    override fun mStart() {
        camera.stopStreaming()
    }

    override fun mLoop() {
        if (tagOfInterest != null) {
            Logger.addTelemetryLine("Tag snapshot:\n")
            tagToTelemetry(tagOfInterest)
        } else {
            Logger.addTelemetryLine("No tag snapshot available, it was never sighted during the init loop :(")
        }
    }

    override fun mStop() {
        camera.stopStreaming()
    }

    private fun tagToTelemetry(detection: AprilTagDetection?) {
        Logger.addTelemetryLine(String.format("\nDetected tag ID=%d", detection!!.id))
        Logger.addTelemetryLine(
            String.format(
                "Translation X: %.2f feet",
                detection.pose.x * FEET_PER_METER
            )
        )
        Logger.addTelemetryLine(
            String.format(
                "Translation Y: %.2f feet",
                detection.pose.y * FEET_PER_METER
            )
        )
        Logger.addTelemetryLine(
            String.format(
                "Translation Z: %.2f feet",
                detection.pose.z * FEET_PER_METER
            )
        )
        Logger.addTelemetryLine(
            String.format(
                "Rotation Yaw: %.2f degrees", Math.toDegrees(
                    detection.pose.yaw
                )
            )
        )
        Logger.addTelemetryLine(
            String.format(
                "Rotation Pitch: %.2f degrees", Math.toDegrees(
                    detection.pose.pitch
                )
            )
        )
        Logger.addTelemetryLine(
            String.format(
                "Rotation Roll: %.2f degrees", Math.toDegrees(
                    detection.pose.roll
                )
            )
        )
    }

    companion object {
        const val FEET_PER_METER = 3.28084
    }
}

