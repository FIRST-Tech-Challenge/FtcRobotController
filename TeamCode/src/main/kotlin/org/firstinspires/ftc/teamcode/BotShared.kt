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
import org.firstinspires.ftc.teamcode.botmodule.Claw
import org.firstinspires.ftc.teamcode.botmodule.Intake
import org.firstinspires.ftc.teamcode.botmodule.LSD
import org.firstinspires.ftc.teamcode.botmodule.March
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive

// TODO: should this be a superclass? our codebase has grown so much, and we might want to reconsider
/**
 * Shared data.
 * Construct during the init phase. Contains HardwareMap definitions, as well as some other classes like the PixelPlacer and MecanumDrive.
 */
@Suppress("MemberVisibilityCanBePrivate", "RedundantSuppression")
class BotShared(opMode: OpMode) {
    // TODO: i wish the hardware would stop changing so that I could keep the code the same for 5 minutes

    // Get stuff from the hardware map (HardwareMap.get() can be HardwareMap[] in kt)
    val hardwareMap = opMode.hardwareMap!!
    @JvmField val imu:              IMU         =           hardwareMap[IMU          ::class.java,   "imu"       ]
    @JvmField val motorLeftFront:   DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "fl"        ]
    @JvmField val motorRightFront:  DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "fr"        ]
    @JvmField val motorLeftBack:    DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "bl"        ]
    @JvmField val motorRightBack:   DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "br"        ]
    @JvmField val camera:           WebcamName? =   idc {   hardwareMap[WebcamName   ::class.java,   "Webcam 1"  ] }
    @JvmField val motorSlide:       DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "lsd"       ] }
    @JvmField val motorIntakeSpin:  DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "inspin"    ] }
    @JvmField val motorIntakeLift:  DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "inlift"    ] }
    @JvmField val motorTruss:       DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "hang"      ] }
    @JvmField val servoArm:         Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "arm"       ] }
    @JvmField val servoClawLeft:    Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "clawl"     ] }
    @JvmField val servoClawRight:   Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "clawr"     ] }

    @JvmField val march               = camera?.    let {   March(opMode, it)   }
    @JvmField val lsd                 = motorSlide?.let {   LSD(opMode, it)     }
    @JvmField val claw                = if (servoClawLeft   != null && servoClawRight != null)  Claw(opMode, servoClawLeft, servoClawRight      )   else null
    @JvmField val intake              = if (motorIntakeLift != null || motorIntakeSpin != null) Intake(opMode, motorIntakeLift, motorIntakeSpin )   else null

    @JvmField var drive: MecanumDrive? = null

    init {
        // IMU orientation/calibration
        val logo = LogoFacingDirection.UP
        val usb = UsbFacingDirection.LEFT
        val orientationOnRobot = RevHubOrientationOnRobot(logo, usb)
        imu.initialize(IMU.Parameters(orientationOnRobot))
        imu.resetYaw()

        // Drive motor directions **(DO NOT CHANGE THESE!!!)**
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