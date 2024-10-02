package com.acmerobotics.roadrunner.ftc

import com.qualcomm.hardware.sparkfun.SparkFunOTOS
import com.qualcomm.robotcore.hardware.I2cDeviceSynch
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType
import java.util.*
@I2cDeviceType
@DeviceProperties(
    name = "SparkFun OTOS Corrected",
    xmlTag = "SparkFunOTOS2",
    description = "SparkFun Qwiic Optical Tracking Odometry Sensor Corrected"
)
class SparkFunOTOSCorrected(deviceClient: I2cDeviceSynch) : SparkFunOTOS(deviceClient) {
    /**
     * Gets only the position and velocity measured by the
     * OTOS in a single burst read
     * @param pos Position measured by the OTOS
     * @param vel Velocity measured by the OTOS
     */
    fun getPosVel(pos: Pose2D, vel: Pose2D) {
        // Read all pose registers
        val rawData = deviceClient.read(REG_POS_XL.toInt(), 12)

        // Convert raw data to pose units
        pos.set(regsToPose(Arrays.copyOfRange(rawData, 0, 6), INT16_TO_METER, INT16_TO_RAD))
        vel.set(regsToPose(Arrays.copyOfRange(rawData, 6, 12), INT16_TO_MPS, INT16_TO_RPS))
    }

    // Modified version of poseToRegs to fix pose setting issue
    // see https://discord.com/channels/225450307654647808/1246977443030368349/1271702497659977760
    override fun poseToRegs(rawData: ByteArray, pose: Pose2D, xyToRaw: Double, hToRaw: Double) {
        // Convert pose units to raw data
        val rawX = (_distanceUnit.toMeters(pose.x) * xyToRaw).toInt().toShort()
        val rawY = (_distanceUnit.toMeters(pose.y) * xyToRaw).toInt().toShort()
        val rawH = (_angularUnit.toRadians(pose.h) * hToRaw).toInt().toShort()

        // Store raw data in buffer
        rawData[0] = (rawX.toInt() and 0xFF).toByte()
        rawData[1] = ((rawX.toInt() shr 8) and 0xFF).toByte()
        rawData[2] = (rawY.toInt() and 0xFF).toByte()
        rawData[3] = ((rawY.toInt() shr 8) and 0xFF).toByte()
        rawData[4] = (rawH.toInt() and 0xFF).toByte()
        rawData[5] = ((rawH.toInt() shr 8) and 0xFF).toByte()
    }
}