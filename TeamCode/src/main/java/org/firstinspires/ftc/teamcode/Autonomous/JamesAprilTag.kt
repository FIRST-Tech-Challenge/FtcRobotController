package org.firstinspires.ftc.teamcode.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.util.Range
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.DESIRED_DISTANCE
import org.firstinspires.ftc.teamcode.Variables.MAX_AUTO_SPEED
import org.firstinspires.ftc.teamcode.Variables.MAX_AUTO_STRAFE
import org.firstinspires.ftc.teamcode.Variables.MAX_AUTO_TURN
import org.firstinspires.ftc.teamcode.Variables.SPEED_GAIN
import org.firstinspires.ftc.teamcode.Variables.STRAFE_GAIN
import org.firstinspires.ftc.teamcode.Variables.TURN_GAIN
import org.firstinspires.ftc.teamcode.Variables.desiredTag
import org.firstinspires.ftc.teamcode.Variables.drive
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR
import org.firstinspires.ftc.teamcode.Variables.strafe
import org.firstinspires.ftc.teamcode.Variables.targetFound
import org.firstinspires.ftc.teamcode.Variables.turn
import org.firstinspires.ftc.vision.VisionPortal
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import java.util.concurrent.TimeUnit

/*
 * Press and hold the *Left Bumper* to enable the automatic "Drive to target" mode.
 * Under "Drive To Target" mode, the robot has three goals:
 * 1) Turn the robot to always keep the Tag centered on the camera frame. (Use the Target Bearing to turn the robot.)
 * 2) Strafe the robot towards the centerline of the Tag, so it approaches directly in front  of the tag.  (Use the Target Yaw to strafe the robot)
 * 3) Drive towards the Tag to get to the desired distance.  (Use Tag Range to drive the robot forward/backward)
 *
 */
@TeleOp(name = "JamesAprilTag", group = "AprilTag")
class JamesAprilTag: DriveMethods() {
    override fun runOpMode() {

        // Initialize the AprilTag Detection process
        initVision(Variables.VisionProcessors.APRILTAG)

        setManualExposure(6, 250) // Use low exposure time to reduce motion blur

        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream")
        telemetry.addData(">", "Touch Play to start OpMode")
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            addDataToTelemetry()

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                val rangeError = desiredTag!!.ftcPose.range - DESIRED_DISTANCE
                val headingError = desiredTag!!.ftcPose.bearing
                val yawError = desiredTag!!.ftcPose.yaw

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED)
                turn = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN)
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE)
                telemetry.addData(
                    "Auto",
                    "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",
                    drive,
                    strafe,
                    turn
                )
            } else {

                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                drive = -gamepad1.left_stick_y / 2.0 // Reduce drive rate to 50%.
                strafe = -gamepad1.left_stick_x / 2.0 // Reduce strafe rate to 50%.
                turn = -gamepad1.right_stick_x / 3.0 // Reduce turn rate to 33%.
                telemetry.addData(
                    "Manual",
                    "Drive %5.2f, Strafe %5.2f, Turn %5.2f ",
                    drive,
                    strafe,
                    turn
                )
            }
            telemetry.update()

            // Apply desired axes motions to the drivetrain.
            moveRobot(drive, strafe, turn)
            sleep(10)
        }
    }
}