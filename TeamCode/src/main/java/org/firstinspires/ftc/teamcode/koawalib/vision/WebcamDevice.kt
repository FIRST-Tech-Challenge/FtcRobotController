package org.firstinspires.ftc.teamcode.koawalib.vision

import com.asiankoala.koawalib.logger.Logger
import com.asiankoala.koawalib.subsystem.Subsystem
import org.openftc.apriltag.AprilTagDetection

class WebcamDevice(val device: Webcam, private val pipeline: AprilTagDetectionPipeline) : Subsystem() {

    var LEFT = 1
    var MIDDLE = 2
    var RIGHT = 3

    var tagOfInterest: AprilTagDetection? = null

    fun start() {
        val currentDetections: ArrayList<AprilTagDetection> = pipeline.latestDetections
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
            } else {
                Logger.addTelemetryLine("Don't see tag of interest :(")
                if (tagOfInterest == null) {
                    Logger.addTelemetryLine("(The tag has never been seen)")
                } else {
                    Logger.addTelemetryLine("\nBut we HAVE seen the tag before; last seen at:")
                }
            }
        } else {
            Logger.addTelemetryLine("Don't see tag of interest :(")
            if (tagOfInterest == null) {
                Logger.addTelemetryLine("(The tag has never been seen)")
            } else {
                Logger.addTelemetryLine("\nBut we HAVE seen the tag before; last seen at:")
            }
        }
    }

    fun update() {
        if (tagOfInterest != null) {
            Logger.addTelemetryLine("Tag snapshot:\n")
        } else {
            Logger.addTelemetryLine("No tag snapshot available, it was never sighted during the init loop :(")
        }
    }
}