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
import org.firstinspires.ftc.teamcode.botmodule.DroneLauncher
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

    // Drive Motors
    @JvmField val motorRightFront:  DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "frontR"        ]
    @JvmField val motorLeftFront:   DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "frontL"        ]
    @JvmField val motorRightBack:   DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "backR"        ]
    @JvmField val motorLeftBack:    DcMotorEx   =           hardwareMap[DcMotorEx    ::class.java,   "backL"        ]

    //Sensors
    @JvmField val imu:              IMU         =           hardwareMap[IMU          ::class.java,   "imu"       ]
    @JvmField val camera1:           WebcamName? =   idc {   hardwareMap[WebcamName   ::class.java,   "Webcam 1"  ] }


    //Outtake
    @JvmField val motorSlideRight:       DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "slideR"       ] }
    @JvmField val motorSlideLeft:       DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "slideL"       ] }
    @JvmField val servoArmRight:    Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "armR"       ] }
    @JvmField val servoArmLeft:     Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "armL"       ] }
    @JvmField val servoClawRight:   Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "clawR"     ] }
    @JvmField val servoClawLeft:    Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "clawL"     ] }


    //Intake
    @JvmField val motorIntake:  DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "intake"    ] }
    @JvmField val servoIntakeLift:    Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "inlift"     ] }

    //Truss Hang
    @JvmField val motorTruss:       DcMotorEx?  =   idc {   hardwareMap[DcMotorEx    ::class.java,   "hang"      ] }
    @JvmField val servoTrussRight:  Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "trussR"       ] }
    @JvmField val servoTrussLeft:   Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "trussL"       ] }

    //Drone Launch
    @JvmField val servoDroneLaunch:   Servo?      =   idc {   hardwareMap[Servo        ::class.java,   "drone"       ] }





    @JvmField val march               = camera1?.    let {   March(opMode, it)   }
    //@JvmField val lsd                 = motorSlide?.let {   LSD(opMode, it)     }
    @JvmField val claw                = if (servoClawLeft   != null && servoClawRight != null)  Claw(opMode, servoClawLeft, servoClawRight      )   else null
    @JvmField val intake              = if (servoIntakeLift != null || motorIntake != null) Intake(opMode, servoIntakeLift, motorIntake )   else null
    @JvmField val droneLauncher        = if (servoDroneLaunch   != null )  DroneLauncher(opMode, servoDroneLaunch)   else null
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
        motorIntake?.   mode =              RUN_WITHOUT_ENCODER
        // Zero-power behavior
        motorLeftFront.     zeroPowerBehavior = BRAKE
        motorLeftBack.      zeroPowerBehavior = BRAKE
        motorRightFront.    zeroPowerBehavior = BRAKE
        motorRightBack.     zeroPowerBehavior = BRAKE
        motorTruss?.        zeroPowerBehavior = BRAKE
        motorSlideRight?.        zeroPowerBehavior = BRAKE
        motorSlideLeft?. zeroPowerBehavior = BRAKE
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