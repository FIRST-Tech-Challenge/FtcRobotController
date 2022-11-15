package org.firstinspires.ftc.teamcode.drive.roadrunner

import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH
import org.firstinspires.ftc.teamcode.drive.robotcore.IMUController
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.leftOdometryPoseInches
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.RobotConstantsAccessor.rearOdometryPoseInches
import org.firstinspires.ftc.teamcode.library.robot.systems.drive.roadrunner.constants.OdometryConstants.*


class TwoWheelOdometryLocalizer
 constructor (
         leftModule : DcMotorEx,
         rearModule : DcMotorEx,
         private val imuController: IMUController
)
    : TwoTrackingWheelLocalizer(
        listOf(
//                Pose2d(INCH.fromCm(leftXcm), INCH.fromCm(leftYcm), leftAngleDeg.toRadians()),
//                Pose2d(INCH.fromCm(rearXcm), INCH.fromCm(rearYcm), rearAngleDeg.toRadians())
                leftOdometryPoseInches,
                rearOdometryPoseInches
        )
)
{

    private val modules = listOf(leftModule, rearModule)

    private val ticksPerRevolution = 8192
    private val wheelDiameterMm = INCH.fromMm(38.0)

    init {
        modules.forEach {
            it.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            it.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }
    }

//    lateinit var bulkData : RevBulkData

    override fun getWheelPositions(): List<Double> {
        val wheelPositions = modules.map {
            (if (reverseOutput) -1.0 else 1.0) * (it.currentPosition.toDouble() / ticksPerRevolution) * wheelDiameterMm * Math.PI
        }

        print("%% @OdometryLocalizer_REVExt   LEFT=${wheelPositions[0]}, REAR=${wheelPositions[1]}")

        return wheelPositions
    }

    override fun getHeading(): Double {
        return imuController.getHeading()
    }
}