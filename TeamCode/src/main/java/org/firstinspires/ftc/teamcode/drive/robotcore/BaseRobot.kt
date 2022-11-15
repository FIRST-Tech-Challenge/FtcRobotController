package org.firstinspires.ftc.teamcode.drive.robotcore

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.*
import org.firstinspires.ftc.teamcode.drive.roadrunner.HolonomicRR
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.Holonomic
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.HolonomicImpl
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.legacy.OdometryModule
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.HolonomicRR

abstract class BaseRobot(protected val hardwareMap: HardwareMap) {
    // Drivetrain Variables
     @JvmField val frontLeftMotor       : DcMotorEx             = hwInit("frontLeftMotor")
     @JvmField val backLeftMotor        : DcMotorEx             = hwInit("backLeftMotor")
     @JvmField val frontRightMotor      : DcMotorEx             = hwInit("frontRightMotor")
     @JvmField val backRightMotor       : DcMotorEx             = hwInit("backRightMotor")

    // Expansion Hub Variables
     @JvmField val expansionhubs        : List<LynxModule>      = hardwareMap.getAll(LynxModule::class.java).apply { forEach {it.bulkCachingMode = LynxModule.BulkCachingMode.AUTO} }

    // Robot Systems Variables
     @JvmField val holonomic            : Holonomic = HolonomicImpl(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor)

     abstract val holonomicRR           : HolonomicRR?

     protected inline fun <reified T> hwInit(name:String): T = hardwareMap.get(T::class.java, name)

}