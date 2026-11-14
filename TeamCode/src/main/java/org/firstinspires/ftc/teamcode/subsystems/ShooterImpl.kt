package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.hardware.DcMotor

class ShooterImpl2 (private val hw: GerthaHardwareMap) {

    companion object{
        private const val ACCEPTABLE_VELOCITY_DIFF = 30.0
    }

    var target: Double = 0.0
        private set

    val defaultPushThreshold = 200
    var pushThreshold = defaultPushThreshold

    var mode = false

    fun init(){
        hw.pollenShooter.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun tickShooter(){
        val targetMode = (target - hw.pollenShooter.velocity) <= pushThreshold
        if (targetMode != mode) if (targetMode) hw.pollenShooter.velocity = target
        mode = targetMode
        if (!mode)
            hw.pollenShooter.power = 1.0
        return
    }

    fun setTarget(vel: Double){
        target = vel
        mode = false
    }
}