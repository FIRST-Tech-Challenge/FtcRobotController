package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.DESIRED_DISTANCE
import org.firstinspires.ftc.teamcode.Variables.MAX_AUTO_SPEED
import org.firstinspires.ftc.teamcode.Variables.MAX_AUTO_STRAFE
import org.firstinspires.ftc.teamcode.Variables.MAX_AUTO_TURN
import org.firstinspires.ftc.teamcode.Variables.SPEED_GAIN
import org.firstinspires.ftc.teamcode.Variables.STRAFE_GAIN
import org.firstinspires.ftc.teamcode.Variables.TURN_GAIN
import org.firstinspires.ftc.teamcode.Variables.clawToBoard
import org.firstinspires.ftc.teamcode.Variables.desiredTag
import org.firstinspires.ftc.teamcode.Variables.drive
import org.firstinspires.ftc.teamcode.Variables.slideToBoard
import org.firstinspires.ftc.teamcode.Variables.strafe
import org.firstinspires.ftc.teamcode.Variables.t
import org.firstinspires.ftc.teamcode.Variables.targetFound
import org.firstinspires.ftc.teamcode.Variables.turn
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit

@TeleOp(name = "Teleop-1", group = "AprilTag")
class Teleop9999: DriveMethods() {
    override fun runOpMode() {
        //init
        initMotorsSecondBot()
        initVision(Variables.VisionProcessors.APRILTAG)
        setManualExposure(6, 250)

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            addDataToTelemetry()

            if (gamepad1.left_bumper && targetFound) {
                val rangeError = desiredTag!!.ftcPose.range - DESIRED_DISTANCE
                val headingError = desiredTag!!.ftcPose.bearing
                val yawError = desiredTag!!.ftcPose.yaw

                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE)
                telemetry.addData("Auto", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",
                    drive,
                    strafe,
                    turn
                )
            } else {
                drive = -gamepad1.left_stick_y / 2.0
                strafe = -gamepad1.left_stick_x / 2.0
                turn = -gamepad1.right_stick_x / 3.0
                telemetry.addData("Manual", "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",
                    drive,
                    strafe,
                    turn
                )
            }

            if (gamepad1.a) {
                for (detection in aprilTag.detections)  {
                    // Original source data
                    var poseY = detection.rawPose.y;

                    slideToBoard = poseY + .05
                    clawToBoard = .01
                    linearSlideCalc()

                }
            }
            telemetry.update()
            moveRobot(drive, strafe, turn)
            sleep(10)
        }
    }
}