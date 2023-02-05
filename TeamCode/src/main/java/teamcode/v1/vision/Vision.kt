package teamcode.v1.vision

import com.asiankoala.koawalib.subsystem.Subsystem
import com.asiankoala.koawalib.subsystem.vision.KWebcam
import org.openftc.easyopencv.OpenCvCameraRotation
import teamcode.v1.vision.SleevePipeline

class Vision: Subsystem() {
    private val pipeline = SleevePipeline()
    private val webcam = KWebcam(
        "webcam",
        pipeline,
        800,
        448,
        OpenCvCameraRotation.UPSIDE_DOWN
    )
    var zone = Enums.Zones.WTF
        private set

    override fun periodic() {
        zone = pipeline.zone
    }

    fun start() {
        webcam.startStreaming()
    }

    fun stop() {
        webcam.stopStreaming()
        pipeline.release()
    }
}