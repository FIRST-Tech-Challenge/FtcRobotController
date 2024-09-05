package org.firstinspires.ftc.teamcode.common.subsystems

import com.arcrobotics.ftclib.command.SubsystemBase
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.ArrayList

class Apriltag : SubsystemBase() {
    val tagLibrary = AprilTagGameDatabase.getSampleTagLibrary()

    val processor = AprilTagProcessor.Builder()
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setDrawCubeProjection(true)
        .setTagLibrary(tagLibrary)
        .build()

    fun getTags(): ArrayList<AprilTagDetection> {
        return processor.detections
    }
}