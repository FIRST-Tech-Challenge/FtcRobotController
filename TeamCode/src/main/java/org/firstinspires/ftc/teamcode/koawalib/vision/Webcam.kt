package org.firstinspires.ftc.teamcode.koawalib.vision

import com.asiankoala.koawalib.hardware.KDevice
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvPipeline

class Webcam(deviceName: String, pipeline: OpenCvPipeline) : KDevice<WebcamName>(deviceName) {

    val camera: OpenCvCamera

    fun startStreaming() {
        camera.openCameraDeviceAsync(object : AsyncCameraOpenListener {
            override fun onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {}
        })
    }

    fun stopStreaming() {
        camera.stopStreaming()
    }

    init {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier(
            "cameraMonitorViewId",
            "id",
            hardwareMap.appContext.packageName
        )
        camera = OpenCvCameraFactory.getInstance().createWebcam(device, cameraMonitorViewId)
        camera.setPipeline(pipeline)
    }
}