@file:Config
@file:Suppress("MemberVisibilityCanBePrivate")

package org.firstinspires.ftc.teamcodekt.components

import com.acmerobotics.dashboard.config.Config
import com.arcrobotics.ftclib.hardware.SimpleServo
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import ftc.rogue.blacksmith.BlackOp
import ftc.rogue.blacksmith.util.kt.invoke
import org.firstinspires.ftc.teamcode.R
import org.firstinspires.ftc.teamcode.pipelines.AprilTagDetectionPipeline
import org.firstinspires.ftc.teamcode.pipelines.BasePoleDetector
import org.firstinspires.ftc.teamcodekt.components.meta.DeviceNames
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.properties.Delegates

@JvmField
var CAM_FORWARDS = 56.5

class Camera {
    private val camServo = SimpleServo(BlackOp.hwMap, DeviceNames.CAM_SERVO, 0.0, 180.0)

    var targetAngle = CAM_FORWARDS

    private val camera: OpenCvCamera

    val aprilTagDetectionPipeline = AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy)
    val poleDetector = BasePoleDetector(BlackOp.mTelemetry)

    fun lookForwards() {
        targetAngle = CAM_FORWARDS
    }

    fun update() {
        camServo.turnToAngle(targetAngle)
    }

    fun setPipeline(pipeline: OpenCvPipeline) {
        camera.setPipeline(pipeline)
    }

    init {
        camera = OpenCvCameraFactory.getInstance().createWebcam(
            BlackOp.hwMap(DeviceNames.WEBCAM1),
            R.id.cameraMonitorViewId,
        )

        camera.setPipeline(aprilTagDetectionPipeline)

        camera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {
                throw RuntimeException("Error opening camera! Error code $errorCode")
            }
        })
    }

    fun waitForStartWithVision(opmode: LinearOpMode): Int? {
        targetAngle = CAM_FORWARDS
        update()

        val telemetry = BlackOp.mTelemetry

        var numFramesWithoutDetection = 0
        var lastIntID: Int? = null

        while (!opmode.opModeIsActive()) {
            val detections = aprilTagDetectionPipeline.detectionsUpdate

            telemetry.addData("FPS", camera.fps)
            telemetry.addData("Overhead ms", camera.overheadTimeMs)
            telemetry.addData("Pipeline ms", camera.pipelineTimeMs)

            if (detections.size == 0) {
                numFramesWithoutDetection++

                if (numFramesWithoutDetection >= THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_LOW)
                }
            } else {
                numFramesWithoutDetection = 0

                if (detections[0].pose.z < THRESHOLD_HIGH_DECIMATION_RANGE_METERS) {
                    aprilTagDetectionPipeline.setDecimation(DECIMATION_HIGH)
                }

                lastIntID = detections.last().id
                telemetry.addLine("\nDetected tag ID=$lastIntID")
            }

            telemetry.update()
        }

        return lastIntID.takeIf { it in 1..3 }
    }

    companion object {
        // Lens intrinsics (units are in pixels)
        private const val fx = 578.272
        private const val fy = 578.272
        private const val cx = 402.145
        private const val cy = 221.506

        // UNITS ARE METERS
        private const val tagsize = 0.166
        private const val DECIMATION_HIGH = 3f
        private const val DECIMATION_LOW = 2f
        private const val THRESHOLD_HIGH_DECIMATION_RANGE_METERS = 1.0f
        private const val THRESHOLD_NUM_FRAMES_NO_DETECTION_BEFORE_LOW_DECIMATION = 4
    }
}
