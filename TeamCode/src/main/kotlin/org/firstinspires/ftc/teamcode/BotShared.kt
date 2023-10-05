package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.robotcore.eventloop.opmode.OpMode
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

    // Get stuff from the hardware map (HardwareMap.get() can be shorthanded to HardwareMap[] in Kotlin)
    @JvmField var imu: IMU =                      opMode.hardwareMap[IMU::class.java,         "imu"]
    @JvmField var tagCamera: WebcamName =         opMode.hardwareMap[WebcamName::class.java,  "Webcam 1"]
    @JvmField var motorFrontRight: DcMotorEx =    opMode.hardwareMap[DcMotorEx::class.java,   "MotorFrontRight"]
    @JvmField var motorFrontLeft: DcMotorEx =     opMode.hardwareMap[DcMotorEx::class.java,   "MotorFrontLeft"]
    @JvmField var motorBackRight: DcMotorEx =     opMode.hardwareMap[DcMotorEx::class.java,   "MotorBackRight"]
    @JvmField var motorBackLeft: DcMotorEx =      opMode.hardwareMap[DcMotorEx::class.java,   "MotorBackLeft"]
    @JvmField var pixelPlacer: PixelPlacer<WebcamName> = PixelPlacer(opMode, tagCamera)
    @JvmField var drive: MecanumDrive? = null

    init {
        // IMU orientation/calibration
        val logo = LogoFacingDirection.UP
        val usb = UsbFacingDirection.FORWARD
        val orientationOnRobot = RevHubOrientationOnRobot(logo, usb)
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw()

        // MecanumDrive (roadrunner) rotations
        motorFrontLeft.direction = REVERSE
        motorBackLeft.direction = REVERSE
        motorFrontRight.direction = FORWARD
        motorBackRight.direction = FORWARD
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