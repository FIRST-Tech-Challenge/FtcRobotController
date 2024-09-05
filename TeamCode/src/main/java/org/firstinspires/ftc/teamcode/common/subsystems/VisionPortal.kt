package org.firstinspires.ftc.teamcode.common.subsystems

import android.util.Size
import com.arcrobotics.ftclib.command.SubsystemBase
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.VisionProcessor


class VisionPortal(val hwMap: HardwareMap, val camera: String, processors: List<VisionProcessor>) : SubsystemBase() {
    val builder = VisionPortal.Builder()
        .setCamera(hwMap.get(WebcamName::class.java, camera))
        .addProcessors(*processors.toTypedArray())
        .setCameraResolution(Size(640, 480))
        .setStreamFormat(VisionPortal.StreamFormat.YUY2)
        .setAutoStopLiveView(true)

    val visionPortal = builder.build()
}