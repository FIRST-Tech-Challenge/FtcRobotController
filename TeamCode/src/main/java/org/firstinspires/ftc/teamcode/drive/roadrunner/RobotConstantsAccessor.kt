package org.firstinspires.ftc.teamcode.drive.roadrunner

import com.acmerobotics.roadrunner.control.PIDCoefficients
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.trajectory.constraints.DriveConstraints
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.drive.roadrunner.constants.DriveConstantsThinBot
import org.firstinspires.ftc.teamcode.drive.roadrunner.constants.OdometryConstants
import org.firstinspires.ftc.teamcode.library.functions.convertUnit
import org.firstinspires.ftc.teamcode.library.functions.toRadians
import kotlin.math.absoluteValue


object RobotConstantsAccessor {

    /*
        General Methods
     */
    fun load(driveClass: Class<out Any>, odometryClass: Class<out Any>) {
        loadedDriveClass = driveClass
        loadedOdometryClass = odometryClass
    }

    /*
        DriveConstants Access
     */
    var loadedDriveClass : Class<out Any> = DriveConstantsThinBot::class.java
        set(value) { field = value.takeIf { it.name.contains("DriveConstants") } ?: DriveConstantsThinBot::class.java}

    val wheelRadius     : Double            get() = loadedDriveClass.getDeclaredField("WHEEL_RADIUS").getDouble(this)
    val gearRatio       : Double            get() = loadedDriveClass.getDeclaredField("GEAR_RATIO").getDouble(this)
    val trackWidth      : Double            get() = loadedDriveClass.getDeclaredField("TRACK_WIDTH").getDouble(this)


    val maxMotorRPM     : Double            get() = loadedDriveClass.getDeclaredField("MAX_RPM").getDouble(this)
    val ticksPerRev     : Double            get() = loadedDriveClass.getDeclaredField("TICKS_PER_REV").getDouble(this)


    val kV              : Double            get() = loadedDriveClass.getDeclaredField("kV").getDouble(this)
    val kA              : Double            get() = loadedDriveClass.getDeclaredField("kA").getDouble(this)
    val kStatic         : Double            get() = loadedDriveClass.getDeclaredField("kStatic").getDouble(this)

    val kF              : Double            get() = getMotorVelocityF()

    val runUsingEncoder : Boolean           get() = loadedDriveClass.getDeclaredField("RUN_USING_ENCODER").getBoolean(this)
    val motorVelocityPID: PIDCoefficients?  get() = loadedDriveClass.getDeclaredField("MOTOR_VELO_PID").get(this) as PIDCoefficients?


    val baseConstraints : DriveConstraints  get() = loadedDriveClass.getDeclaredField("BASE_CONSTRAINTS").get(this) as DriveConstraints


    val translationalXPID:PIDCoefficients   get() = loadedDriveClass.getDeclaredField("TRANSLATIONAL_X_PID").get(this) as PIDCoefficients
    val translationalYPID:PIDCoefficients   get() = loadedDriveClass.getDeclaredField("TRANSLATIONAL_Y_PID").get(this) as PIDCoefficients
    val headingPID      :PIDCoefficients    get() = loadedDriveClass.getDeclaredField("HEADING_PID").get(this) as PIDCoefficients

    fun rpmToVelocity(rpm: Double)          = rpm * gearRatio * 2 * Math.PI * wheelRadius / 60.0
    fun getMaxRPM()                         = maxMotorRPM * 0.85
    fun encoderTicksToInches(ticks: Double) = wheelRadius * 2 * Math.PI * gearRatio * ticks / 145.6
    fun getTicksPerSec()                    = maxMotorRPM * ticksPerRev / 60.0
    fun getMotorVelocityF()                 = 32767 / getTicksPerSec()


    /*
        OdometryConstants Access
     */
    var loadedOdometryClass : Class<out Any> = OdometryConstants::class.java
        set(value) { field = value.takeIf { it.name.contains("OdometryConstants") } ?: OdometryConstants::class.java}

    val leftOdometryPoseInches : Pose2d
        get() =
            Pose2d(loadedOdometryClass.getDeclaredField("leftXcm").getDouble(this).convertUnit(DistanceUnit.CM, DistanceUnit.INCH),
                   loadedOdometryClass.getDeclaredField("leftYcm").getDouble(this).convertUnit(DistanceUnit.CM, DistanceUnit.INCH),
                   loadedOdometryClass.getDeclaredField("leftAngleDeg").getDouble(this).toRadians())
    val rightOdometryPoseInches : Pose2d
        get() =
            Pose2d(loadedOdometryClass.getDeclaredField("rightXcm").getDouble(this).convertUnit(DistanceUnit.CM, DistanceUnit.INCH),
                   loadedOdometryClass.getDeclaredField("rightYcm").getDouble(this).convertUnit(DistanceUnit.CM, DistanceUnit.INCH),
                   loadedOdometryClass.getDeclaredField("rightAngleDeg").getDouble(this).toRadians())
    val rearOdometryPoseInches : Pose2d
        get() =
            Pose2d(loadedOdometryClass.getDeclaredField("rearXcm").getDouble(this).convertUnit(DistanceUnit.CM, DistanceUnit.INCH),
                   loadedOdometryClass.getDeclaredField("rearYcm").getDouble(this).convertUnit(DistanceUnit.CM, DistanceUnit.INCH),
                   loadedOdometryClass.getDeclaredField("rearAngleDeg").getDouble(this).toRadians())
    val odometryLateralDistance: Double
        get() =
            (loadedOdometryClass.getDeclaredField("leftYcm").getDouble(this) - loadedOdometryClass.getDeclaredField("rightYcm").getDouble(this))
                    .absoluteValue
}