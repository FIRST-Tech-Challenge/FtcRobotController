package org.firstinspires.ftc.teamcode.koawalib.vision

import com.asiankoala.koawalib.hardware.KDevice
import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.util.Periodic
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.apriltag.AprilTagDetection
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCamera.AsyncCameraOpenListener
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

@TeleOp
class Webcam(deviceName: String) : KDevice<WebcamName>(deviceName) {

     private val camera: OpenCvCamera

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
        camera.setPipeline(SleevePipeline(0.166, 578.272, 578.272, 402.145, 221.506))
    }
    }
