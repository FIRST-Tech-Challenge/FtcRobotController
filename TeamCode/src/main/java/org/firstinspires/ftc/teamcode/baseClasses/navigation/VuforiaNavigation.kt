package org.firstinspires.ftc.teamcode.baseClasses.navigation

import com.acmerobotics.dashboard.FtcDashboard
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.ClassFactory
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection
import org.firstinspires.ftc.teamcode.baseClasses.*
import kotlin.collections.ArrayList
import kotlin.concurrent.thread

@Suppress("PrivatePropertyName")
class VuforiaNavigation(logger: LoggerFunction) {
    private val logger: LoggerFunction = logger

    private val CAMERA_DIRECTION: VuforiaLocalizer.CameraDirection = VuforiaLocalizer.CameraDirection.BACK
    private val PHONE_IS_PORTRAIT = false
    private val VUFORIA_KEY = "ATo2Fcb/////AAABmdzs1kF7l03Wi+1A4+h9Gb8uBv/aY255UIi4TcFhJnqe6882bPbdwOMU1DllP2tR/SxA5C0RCsa2Q4YEHwx1Q7XStreON6ORBRV1c1BTKp3OAgfQfJwDdqZN0Qk+fDB2JDt8RwZ5/NRNJEwo8aW4slGfuw33MmQ+YQN2kyypkvNN6OwoMxUK5bmPC5M/3EVUWtcH5tU9YiYoJ5H085WFHEbzJHPrX2lVsdBq+hDB+Sf8HFQefprbaDf06BDGEstU0EC/45aihspR1MCt8Dm98qbAdD75Jk5BIr8pRtdEZAmefwTQnYu2ouZkTV/JOp5xOPq322UcQVRoflB7ypQ9konbTPSJmp5L3VVdX1EMCd86";

    private val TARGET_HEIGHT = UnitDistance.inches(6).millimetres.toFloat()

    // Perimeter targets
    private val HALF_FIELD_PERIMETER_TARGET = UnitDistance.inches(72).millimetres.toFloat()
    private val QUARTER_FIELD_PERIMETER_TARGET = UnitDistance.inches(36).millimetres.toFloat()

    private var lastLocation: OpenGLMatrix? = null
    private lateinit var vuforia: VuforiaLocalizer
    private var targetVisible = false

    // Phone Position
    private val phoneXRotation = if (PHONE_IS_PORTRAIT) 90f else 0f // Rotate the phone vertical about the X axis if it's in portrait mode
    private val phoneYRotation = if (CAMERA_DIRECTION == CameraDirection.BACK) -90f else 90f // We need to rotate the camera around it's long axis to bring the correct camera forward.
    private val phoneZRotation = 0f

    private val CAMERA_FORWARD_DISPLACEMENT = UnitDistance.inches(4).millimetres.toFloat() // eg: Camera is 4 Inches in front of robot center
    private val CAMERA_VERTICAL_DISPLACEMENT = UnitDistance.inches(8).millimetres.toFloat() // eg: Camera is 8 Inches above ground
    private val CAMERA_LEFT_DISPLACEMENT = UnitDistance.inches(0).millimetres.toFloat() // eg: Camera is ON the robot's center line

    private var shouldUpdateLocation = true
    private var shouldLog = false

    private lateinit var hardwareMap: HardwareMap
    private lateinit var vuforiaTargets: VuforiaTrackables
    private lateinit var targetsList: ArrayList<VuforiaTrackable>
    private lateinit var vuforiaParams: VuforiaLocalizer.Parameters
    private lateinit var cameraToRobotTransformationMatrix: OpenGLMatrix

    private lateinit var blueTowerGoalTarget: VuforiaTrackable
    private lateinit var redTowerGoalTarget: VuforiaTrackable
    private lateinit var redAllianceTarget: VuforiaTrackable
    private lateinit var blueAllianceTarget: VuforiaTrackable
    private lateinit var frontWallTarget: VuforiaTrackable

    fun init(hardwareMap: HardwareMap, logging: Boolean = false) {
        this.hardwareMap = hardwareMap
        shouldLog = logging

        logger { log ->
            log.text("Vuforia :: Initializing")
            log.text("Vuforia Logging", logging.toString())
        }

        initVuforia()
        loadTargets()
        positionTargets()
        getRobotTranslationMatrix()
        sendRobotTranslationMatrixToTrackables()
        activateTargets()
        startLocationUpdates()
    }


    private fun initVuforia() {
        val cameraMonitorID = hardwareMap.appContext.resources.getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.packageName)
        val parameters = VuforiaLocalizer.Parameters(cameraMonitorID)

        parameters.vuforiaLicenseKey = VUFORIA_KEY
        parameters.cameraDirection = CAMERA_DIRECTION
        parameters.useExtendedTracking = false

        vuforia = ClassFactory.getInstance().createVuforia(parameters)

        vuforiaParams = parameters

        FtcDashboard.getInstance().startCameraStream(vuforia, 0.0)
    }


    private fun loadTargets() {
        val ultimateGoalTargets = vuforia.loadTrackablesFromAsset("UltimateGoal")
        blueTowerGoalTarget = ultimateGoalTargets[0]
        blueTowerGoalTarget.name = "Blue Tower Goal Target"
        redTowerGoalTarget = ultimateGoalTargets[1]
        redTowerGoalTarget.name = "Red Tower Goal Target"
        redAllianceTarget = ultimateGoalTargets[2]
        redAllianceTarget.name = "Red Alliance Target"
        blueAllianceTarget = ultimateGoalTargets[3]
        blueAllianceTarget.name = "Blue Alliance Target"
        frontWallTarget = ultimateGoalTargets[4]
        frontWallTarget.name = "Front Wall Target"

        val allTrackables: java.util.ArrayList<VuforiaTrackable> = java.util.ArrayList()
        allTrackables.addAll(ultimateGoalTargets)

        vuforiaTargets = ultimateGoalTargets
        targetsList = allTrackables
    }


    private fun positionTargets() {
        // Position targets relative to field origin
        redAllianceTarget.location = OpenGLMatrix
                .translation(0f, -HALF_FIELD_PERIMETER_TARGET, TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, 180f))
        blueAllianceTarget.location = OpenGLMatrix
                .translation(0f, HALF_FIELD_PERIMETER_TARGET, TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, 0f))
        frontWallTarget.location = OpenGLMatrix
                .translation(-HALF_FIELD_PERIMETER_TARGET, 0f, TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, 90f))

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.location = OpenGLMatrix
                .translation(HALF_FIELD_PERIMETER_TARGET, QUARTER_FIELD_PERIMETER_TARGET, TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, -90f))
        redTowerGoalTarget.location = OpenGLMatrix
                .translation(HALF_FIELD_PERIMETER_TARGET, -QUARTER_FIELD_PERIMETER_TARGET, TARGET_HEIGHT)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90f, 0f, -90f))
    }


    private fun getRobotTranslationMatrix() {
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        val matrix = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(AxesReference.EXTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES, phoneYRotation, phoneZRotation, phoneXRotation))

        cameraToRobotTransformationMatrix = matrix
    }


    private fun sendRobotTranslationMatrixToTrackables() {
        targetsList.forEach { target ->
            (target.listener as VuforiaTrackableDefaultListener).setPhoneInformation(cameraToRobotTransformationMatrix, vuforiaParams.cameraDirection)
        }
    }


    private fun activateTargets() {
        vuforiaTargets.activate()
    }


    private fun startLocationUpdates() {
        thread {
            while (shouldUpdateLocation) {
                logger { log ->
                    getRobotPosition(log)
                    if (shouldLog) printRobotPosition(log)
                }
            }

            logger { log -> log.text("Vuforia :: Finished updating location") }
        }
    }


    private fun getRobotPosition(log: Logger) {
        targetVisible = false
        targetsList.forEach { target ->
            if ((target.listener as VuforiaTrackableDefaultListener).isVisible) {
                if (shouldLog) log.text("Visible Target", target.name)
                targetVisible = true

                val robotLocationTransform = (target.listener as VuforiaTrackableDefaultListener).updatedRobotLocation
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform
                    return
                }
            }
        }
    }


    private fun printRobotPosition(log: Logger) {
        lastLocation?.also { location ->
            val translation = location.translation
            val orientation = Orientation.getOrientation(location, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES)

//            log("Translation (mm) {x, y, z} :: ${translation[0]}, ${translation[1]}, ${translation[2]}")
//            log("Orientation (deg) {x, y, z} :: ${orientation.firstAngle}, ${orientation.secondAngle}, ${orientation.thirdAngle}")

            log.text("Translation X", translation[0].toString())
            log.text("Translation Y", translation[1].toString())
            log.text("Translation Z", translation[2].toString())

            log.location(location)
        }
    }


    fun getLastKnownRobotPosition(): OpenGLMatrix? {
        return lastLocation
    }


    fun deinit() {
        shouldUpdateLocation = false
        vuforiaTargets.deactivate()
    }
}