package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.Pose2d
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.FORWARD
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

/**
 * Shared data.
 * Construct during the init phase. Contains HardwareMap definitions, as well as some other classes like the PixelPlacer and MecanumDrive.
 */
@Suppress("MemberVisibilityCanBePrivate", "RedundantSuppression")
class BotShared(opMode: OpMode) {
    // TODO: i wish the hardware would stop changing so that I could keep the code the same for 5 minutes

    // Get stuff from the hardware map (HardwareMap.get() can be HardwareMap[] in kt)
    val hardwareMap = opMode.hardwareMap!!;
    @JvmField val imu:              IMU         =           hardwareMap[IMU          ::class.java,   "imu"       ]
    @JvmField val camera:           WebcamName? =   idc {   hardwareMap[WebcamName   ::class.java,   "Webcam 1"  ] }
    @JvmField val motorLeftFront:   DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "fl"        ]
    @JvmField val motorRightFront:  DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "fr"        ]
    @JvmField val motorLeftBack:    DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "bl"        ]
    @JvmField val motorRightBack:   DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "br"        ]
    @JvmField val motorSlide:       DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "lsd"       ] }
    @JvmField val motorIntakeSpin:  DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "inspin"    ] }
    @JvmField val motorIntakeLift:  DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "inlift"    ] }
    @JvmField val motorTruss:       DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "hang"      ] }
    @JvmField val servoArm:         Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "arm"       ] }

    @JvmField var drive: MecanumDrive? = null

    @JvmField var march =                 camera?.let { March(opMode, it) }
    @JvmField var lsd =               motorSlide?.let { LSD(opMode, it) }
    @JvmField var intake =       if (motorIntakeLift != null || motorIntakeSpin != null ) Intake(opMode, motorIntakeLift, motorIntakeSpin)  else null

    init {
        // IMU orientation/calibration
        val logo = LogoFacingDirection.UP
        val usb = UsbFacingDirection.LEFT
        val orientationOnRobot = RevHubOrientationOnRobot(logo, usb)
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw()

        // Motor directions **(DO NOT CHANGE THESE!!!)**
        motorLeftFront.     direction =         REVERSE
        motorLeftBack.      direction =         REVERSE
        motorRightFront.    direction =         FORWARD
        motorRightBack.     direction =         FORWARD
        // Modes
        motorTruss?.        mode =              RUN_WITHOUT_ENCODER
        motorIntakeSpin?.   mode =              RUN_WITHOUT_ENCODER
        // Zero-power behavior
        motorLeftFront.     zeroPowerBehavior = BRAKE
        motorLeftBack.      zeroPowerBehavior = BRAKE
        motorRightFront.    zeroPowerBehavior = BRAKE
        motorRightBack.     zeroPowerBehavior = BRAKE
        motorTruss?.        zeroPowerBehavior = BRAKE
        motorSlide?.        zeroPowerBehavior = BRAKE
    }
    /**
     * Should be called every update.
     * Place any reusable update functions here (i.e. for MecanumDrive)
     */
    fun update() {
        drive?.updatePoseEstimate()
    }

    companion object {
        /**
         * RoadRunner Pose Storage
         */
        @JvmStatic var storedPose: Pose2d = Pose2d(0.0, 0.0, 0.0)

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