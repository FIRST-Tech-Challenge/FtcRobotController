package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.hardware.DcMotor
import kotlin.math.abs
import kotlin.reflect.KProperty

class ShooterImpl (private val hw: GerthaHardwareMap) {

    companion object{
        private const val ACCEPTABLE_VELOCITY_DIFF = 30.0

        var KP = 16.0
        var KD = 3.5
        var KI = 1.5
    }

    var target: Double = 0.0
        private set

    val defaultPushThreshold = 200
    var pushThreshold = defaultPushThreshold

    var mode = false

    var prevTime = System.nanoTime() / 1_000_000
    var prevError = 0.0

    fun init(){
        hw.pollenShooter.mode = DcMotor.RunMode.RUN_USING_ENCODER
    }

    fun tickShooterOld(){
        val targetMode = (target - hw.pollenShooter.velocity) <= pushThreshold
        if (targetMode != mode) if (targetMode) hw.pollenShooter.velocity = target
        mode = targetMode
        if (!mode)
            hw.pollenShooter.power = 1.0
        return
    }

    fun tickShooter() {
        val currentTime = System.nanoTime() / 1_000_000
        val currentVel = hw.pollenShooter.velocity
        val error = target - currentVel

        val p = error * KP

        val i = (error * (currentTime - prevTime)) * KI

        val d = ((error - prevError) / (currentTime - prevTime)) * KD

        if (error > 50) {
            hw.pollenShooter.power = 1.0
        }
        else {
            Log.i("shooterPID", (p + i + d).toString())
            hw.pollenShooter.power = (p + i + d)
        }

        prevTime = currentTime
        prevError = error



    }

    fun setTarget(vel: Double){
        target = vel
        mode = false
    }
}