package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.VoltageSensor
import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap
import kotlin.math.abs
import kotlin.reflect.KProperty
import kotlin.toString

class ShooterImpl (private val hw: GerthaHardwareMap) {

    companion object{
        private const val ACCEPTABLE_VELOCITY_DIFF = 30.0

//        private const val maxVoltage = 12.0
//
//        private const val maxVelocity = 1820.0

        private const val KF = 0.0071
        var KP = 0.004
        var KD = 0.002
        var KI = 0.0
    }

    var target: Double = 0.0
        private set

    var targetInRPM = 0.0

    public var f = 0.0
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
        if (targetMode != mode) if (targetMode) hw.setPollenVelocity(target)
        mode = targetMode
        if (!mode)
            hw.pollenShooter.power = 1.0
        Log.i("shooter", hw.pollenShooter.power.toString())
        return
    }

    fun tickShooter() {
        val currentVoltage = hw.voltageSensor.voltage
        val targetVoltage = KF * target
        val currentTime = System.nanoTime() / 1_000_000
        val currentVel = hw.pollenShooter.velocity
        val error = target - currentVel
        Log.i("Error", (-(error)).toString())


        val p = error * KP

        val i = (error * (currentTime - prevTime)) * KI

        val d = ((error - prevError) / (currentTime - prevTime)) * KD

        f = targetVoltage / currentVoltage
        
//        val targetMode = (target - hw.pollenShooter.velocity) <= pushThreshold
//        if (targetMode != mode){
//            if (targetMode){
//                hw.pollenShooter.power = (p + i + d + f)
//            }
//        }
//        mode = targetMode
//        if(!mode) {
//            hw.pollenShooter.power = 1.0
//        }

        if (error > hw.PUSH_THRESHOLD) {
            hw.pollenShooter.power = 1.0
        }
        else {
            hw.pollenShooter.power = (p + i + d + f)
        }
//        hw.pollenShooter.power = (p + i + d + f)

        prevTime = currentTime
        prevError = error
    }

    fun setTarget(vel: Double){
        target = vel
        mode = false
    }
}