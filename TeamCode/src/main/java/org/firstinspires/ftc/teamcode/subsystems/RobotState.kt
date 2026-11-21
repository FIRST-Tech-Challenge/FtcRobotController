package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.hardware.RobotPose
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import kotlin.math.abs
import kotlin.math.hypot

class RobotState(private val hw: GerthaHardwareMap, val isRed: Boolean = true, val liveUpdate: Boolean = true) {

    var ppX: Double = 0.0
    var ppY: Double = 0.0
    var ppThetaDeg: Double = 0.0
    var ppThetaRad: Double = 0.0
    var ppRobotPose = RobotPose(ppX, ppY, ppThetaRad)


    var velXM: Double = 0.0
    var velYM: Double = 0.0

    var velXInch: Double = 0.0
    var velYInch: Double = 0.0
    var velInch: Double = 0.0

    var angVelRad: Double = 0.0

    val aprilTag = AprilTagProcessor.easyCreateWithDefaults()
    val visionPortal = VisionPortal.easyCreateWithDefaults(hw.webcam1, aprilTag)


    var rightHive = true

    var isNotEmpty = true
    var onlySeesOtherColor = false

    var red = 0
    var blue = 0
    var green = 0

    fun init(ppStartPose: RobotPose){
//        hw.pinpoint.resetPosAndIMU()
        hw.pinpoint.position = ppStartPose.asPose2D
        val ppPose = hw.pinpoint.position
        ppX = ppPose.getX(DistanceUnit.INCH)
        ppY = ppPose.getY(DistanceUnit.INCH)
        ppThetaRad = ppPose.getHeading(AngleUnit.RADIANS)
        ppThetaDeg = ppPose.getHeading(AngleUnit.DEGREES)
        ppRobotPose = RobotPose(ppX, ppY, ppThetaRad)
    }

    fun update() {
        hw.pinpoint.update()

        val ppPose = hw.pinpoint.position
        ppX = ppPose.getX(DistanceUnit.INCH)
        ppY = ppPose.getY(DistanceUnit.INCH)
        ppThetaRad = ppPose.getHeading(AngleUnit.RADIANS)
        ppThetaDeg = ppPose.getHeading(AngleUnit.DEGREES)
        ppRobotPose = RobotPose(ppX, ppY, ppThetaRad)


        velXM = hw.pinpoint.getVelX(DistanceUnit.METER)
        velYM = hw.pinpoint.getVelY(DistanceUnit.METER)

        velXInch = hw.pinpoint.getVelX(DistanceUnit.INCH)
        velYInch = hw.pinpoint.getVelY(DistanceUnit.INCH)
        velInch = hypot(velXInch, velYInch)

        angVelRad = hw.pinpoint.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS)

        red = hw.color.red()
        blue = hw.color.blue()
        green = hw.color.green()

        val currentDetections: MutableList<AprilTagDetection> = aprilTag.getDetections()

        isNotEmpty = currentDetections.isNotEmpty()

        if (isNotEmpty && abs(ppRobotPose.x) > 36 && liveUpdate){
            var count = 0
            for (detection in currentDetections) {
                if (detection is AprilTagClusterDetection && detection.ftcPose != null) {

                    val clusterDet = detection

                    val roll = clusterDet.ftcPose.roll
                    var pitch = clusterDet.ftcPose.pitch

                    val clusterName = clusterDet.metadata.name
                    Log.i("name", clusterName)

                    val inverted = abs(roll) > 90
                    if (inverted){
                        pitch = -pitch
                    }

                    //pitch<0 means that cluster is up
                    //pitch<0 means that cluster is down

                    if (isRed) {
                        if (clusterName.lowercase().contains("blue")) {
                            continue
                        }
                    }
                    if (!isRed) {
                        if (clusterName.lowercase().contains("red")) {
                            continue
                        }
                    }

                    count++
                    if (clusterName.lowercase().contains("audience")){
                        if (pitch > 0 && isRed) {
                            Log.i("hive", "a")
                            rightHive = true
                        } else if (pitch > 0 && !isRed){
                            Log.i("hive", "b")
                            rightHive = false
                        } else if (pitch < 0 && isRed){
                            Log.i("hive", "c")
                            rightHive = false
                        } else if (pitch < 0 && !isRed){
                            Log.i("hive", "d")
                            rightHive = true
                        }
                    } else if (clusterName.lowercase().contains("scoring")){
                        if (pitch > 0 && isRed) {
                            Log.i("hive", "d")
                            rightHive = false
                        } else if (pitch > 0 && !isRed){
                            Log.i("hive", "e")
                            rightHive = true
                        } else if (pitch < 0 && isRed){
                            Log.i("hive", "f")
                            rightHive = true
                        } else if (pitch < 0 && !isRed){
                            Log.i("hive", "g")
                            rightHive = false
                        }
                    }
                }
        }
            if (count > 0) {
                onlySeesOtherColor = false
            }
            else {
                onlySeesOtherColor = true
            }

        }


    }

}