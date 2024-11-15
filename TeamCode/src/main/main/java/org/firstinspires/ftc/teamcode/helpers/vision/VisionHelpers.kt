package org.firstinspires.ftc.teamcode.helpers.vision

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvPipeline
import java.util.concurrent.TimeUnit.MILLISECONDS


typealias Exposure = Long
typealias Gain = Int
typealias ViewId = Int

private val aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults()

fun visionPortal(camera: CameraName): VisionPortal =
    VisionPortal.Builder().setCamera(camera).addProcessor(aprilTagProcessor).build()
val VisionPortal.aprilTagProcessor: AprilTagProcessor
    get() = this.aprilTagProcessor

val VisionPortal.exposureControl: ExposureControl
    get() = this.getCameraControl<ExposureControl>(ExposureControl::class.java)
val VisionPortal.gainControl: GainControl
    get() = this.getCameraControl<GainControl>(GainControl::class.java)

var VisionPortal.exposure: Exposure
    get() = this.exposureControl.getExposure(MILLISECONDS)
    set(value) {
        this.exposureControl.setExposure(value, MILLISECONDS)
    }

var VisionPortal.gain: Gain
    get() = this.gainControl.gain
    set(value) {
        this.gainControl.gain = value
    }


var OpenCvCamera.pipeline: OpenCvPipeline
    get() {
        TODO()
    }
    set(value) {
        this.setPipeline(value)
    }

fun openCvCamera(camera: WebcamName, viewId: ViewId): OpenCvCamera {
    return OpenCvCameraFactory.getInstance().createWebcam(camera, viewId)
}