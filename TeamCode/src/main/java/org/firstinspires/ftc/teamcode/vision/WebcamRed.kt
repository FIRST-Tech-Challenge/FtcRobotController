package org.firstinspires.ftc.teamcode.vision

import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraFactory
import org.openftc.easyopencv.OpenCvCameraRotation

class WebcamRed : Subsystem{
    private lateinit var webcam : OpenCvCamera
    lateinit var pipeline : PipelineRed


    fun init(hardwareMap: HardwareMap) {
        val cameraMonitorViewId = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val webcamName = hardwareMap[WebcamName::class.java, "Webcam"]
        webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId)

        pipeline = PipelineRed()
        webcam.setPipeline(pipeline)

        webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT)
            }

            override fun onError(errorCode: Int) {

            }
        })

    }


    override fun sendDashboardPacket(debugging: Boolean) {

    }

    override fun update() {
        if (pipeline.LeftTotal > pipeline.MIN_R && pipeline.RightTotal > pipeline.MIN_R && pipeline.CenterTotal > pipeline.MIN_R) {
            //Left is TSE
            pipeline.cupState = PipelineRed.CupStates.LEFT
        }

        else if ( pipeline.LeftTotal > pipeline.MIN_R && pipeline.CenterTotal < pipeline.RightTotal) {
            //Center is TSE
            pipeline.cupState = PipelineRed.CupStates.CENTER
        }

        else if ( pipeline.RightTotal < pipeline.CenterTotal && pipeline.LeftTotal > pipeline.MIN_R) {
            //Right is TSE
            pipeline.cupState = PipelineRed.CupStates.RIGHT
        }

        else {
            pipeline.cupState = PipelineRed.CupStates.RIGHT
        }
    }

    override fun reset() {
        webcam.stopStreaming()
    }

}