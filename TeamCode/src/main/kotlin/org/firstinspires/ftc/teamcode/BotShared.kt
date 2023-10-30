package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

/**
 * Shared data.
 * Construct during the init phase. Contains HardwareMap definitions, as well as some other classes like the PixelPlacer and MecanumDrive.
 */
@Suppress("MemberVisibilityCanBePrivate", "RedundantSuppression")
class BotShared(opMode: OpMode) {

    // Get stuff from the hardware map (HardwareMap.get() can be HardwareMap[] in kt)
    @JvmField var imu: IMU =                            opMode.hardwareMap[IMU::class.java,         "imu"]
    @JvmField var camera: WebcamName? =         try {   opMode.hardwareMap[WebcamName::class.java,  "Webcam 1"]  } catch (_: Exception) { null }
    @JvmField var motorLeftFront: DcMotorEx =           opMode.hardwareMap[DcMotorEx::class.java,   "fl"]
    @JvmField var motorRightFront: DcMotorEx =          opMode.hardwareMap[DcMotorEx::class.java,   "fr"]
    @JvmField var motorLeftBack: DcMotorEx =            opMode.hardwareMap[DcMotorEx::class.java,   "bl"]
    @JvmField var motorRightBack: DcMotorEx =           opMode.hardwareMap[DcMotorEx::class.java,   "br"]
    @JvmField var motorSlide: DcMotorEx? =      try {   opMode.hardwareMap[DcMotorEx::class.java,   "lsd"] } catch (_: Exception) { null }
    @JvmField var motorIntakeSpin: DcMotorEx =          opMode.hardwareMap[DcMotorEx::class.java,   "inspin"]
    @JvmField var motorIntakeLift: DcMotorEx? = try {   opMode.hardwareMap[DcMotorEx::class.java,   "inlift"] } catch (_: Exception) { null }

    @JvmField var drive: MecanumDrive? = null

    @JvmField var march =                 camera?.let { March(opMode, it) }
    @JvmField var lsd =               motorSlide?.let { LSD(opMode, it) }
    @JvmField var intake =       motorIntakeLift?.let { PixelIntake(opMode, it, motorIntakeSpin) }

    init {
        // IMU orientation/calibration
        val logo = LogoFacingDirection.UP
        val usb = UsbFacingDirection.RIGHT
        val orientationOnRobot = RevHubOrientationOnRobot(logo, usb)
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw()

        // Motor rotations (DO NOT CHANGE THESE!!!)
        motorLeftFront. direction = REVERSE
        motorLeftBack.  direction = REVERSE
        motorRightFront.direction = FORWARD
        motorRightBack. direction = FORWARD
        motorLeftFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorLeftBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorRightFront.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorRightBack.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }
    /**
     * Should be called every update.
     * Place any reusable update functions here (i.e. for MecanumDrive)
     */
    fun update() {
        drive?.updatePoseEstimate()
    }

    /** RoadRunner Pose Storage */
    companion object {
        @JvmStatic
        var storedPose: Pose2d = Pose2d(0.0, 0.0, 0.0);
    }

}

// Was originally going to be a subclass of OpMode, but supers are a pain, so it's an object instead.
//@Suppress("MemberVisibilityCanBePrivate")
//open class HawkShared: OpMode() {
//    // Get stuff from the hardware map (HardwareMap.get() can be shorthanded to HardwareMap[] in Kotlin)
//    lateinit var tagCamera: WebcamName
//    lateinit var motorFrontRight: DcMotorEx
//    lateinit var motorFrontLeft: DcMotorEx
//    lateinit var motorBackRight: DcMotorEx
//    lateinit var motorBackLeft: DcMotorEx
//    lateinit var pixelPlacer: PixelPlacer<WebcamName>
//    var drive: MecanumDrive? = null
//
//    override fun init() {
//        tagCamera =         this.hardwareMap[WebcamName::class.java,    "Webcam 1"]
//        motorFrontRight =   this.hardwareMap[DcMotorEx::class.java,     "MotorFrontRight"]
//        motorFrontLeft =    this.hardwareMap[DcMotorEx::class.java,     "MotorFrontLeft"]
//        motorBackRight =    this.hardwareMap[DcMotorEx::class.java,     "MotorBackRight"]
//        motorBackLeft =     this.hardwareMap[DcMotorEx::class.java,     "MotorBackLeft"]
//        pixelPlacer =       PixelPlacer(this, tagCamera)
//    }
//
//    override fun loop() {}
//}