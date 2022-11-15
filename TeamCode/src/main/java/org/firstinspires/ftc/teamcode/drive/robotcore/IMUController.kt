package org.firstinspires.ftc.teamcode.drive.robotcore

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.*

class IMUController @JvmOverloads constructor(hardwareMap: HardwareMap, private val axesOrder: AxesOrder = AxesOrder.ZYX, id: Char) {
    @JvmField val imuA          : BNO055IMU = hardwareMap.get(BNO055IMU::class.java, "imu$id")

    init {
        fun initImu(imu: BNO055IMU, id:Char) {
            val parameters = BNO055IMU.Parameters()
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
            parameters.calibrationDataFile = "IMU_${id}_CalibrationData.json" // see the calibration sample opmode
            parameters.loggingEnabled = true
            parameters.loggingTag = "IMU_${id}"
            parameters.accelerationIntegrationAlgorithm = JustLoggingAccelerationIntegrator()
            imu.initialize(parameters)
            imu.startAccelerationIntegration(Position(), Velocity(), 1000)
        }

        initImu(imuA, 'A')
    }

    /**
     * Function to simplify getting the current heading from the IMU
     *
     * @return Current robot heading, in radians
     */
    fun getHeading() = imuA.getAngularOrientation(AxesReference.INTRINSIC, axesOrder, AngleUnit.RADIANS).firstAngle.toDouble()
}