package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.*
import kotlin.math.abs

class IntakeShootMachine(private val hw: GerthaHardwareMap) {

    //TODO first loop justTransitioned, switch to continuous??

    enum class State {
        OFF,
        ON,
        OUTTAKE,
        PRE_SHOOT,
        SHOOT
    }

    lateinit var state: State
    lateinit var prevState: State

    var count = 0
    var continuous = false;

    var justTransitioned = true

    companion object {
        private val INTAKE_POWER = 1.0
        private val OUTTAKE_POWER = -1.0
        private val SHOOT_POWER = 1.0
    }

    private var conditionContiniousStartTimeNs: Long = 0L
    private var stateStartTimeNs: Long = 0L

    private var lastDistance = 0.0

    fun init(){
        state = State.OFF
        prevState = State.ON
        hw.pollenBallStop.position = POLLEN_STOP
        //??
    }

    fun update(robotState: RobotState, gamepadState: GamepadState){

        justTransitioned = (state != prevState)
        if (justTransitioned){
            clearTimers()
        }
        prevState = state

        when (state) {
            State.OFF -> {
                if (justTransitioned) {
                    hw.setIntakePower(0.0)
                }
                if (gamepadState.rb1 && count < 4){
                    state = State.ON
                } else if (gamepadState.lb1) {
                    state = State.OUTTAKE
                } else if (gamepadState.b2) {
                    state = State.PRE_SHOOT
                }
            }
            State.ON -> {
                val distance = hw.color.getDistance(DistanceUnit.CM)
                if (justTransitioned) {
                    hw.pollenBallStop.position = POLLEN_STOP
                    hw.setIntakePower(INTAKE_POWER)
                }
                if(distance < DISTANCE_THRESHOLD && !continuous) {
                    count++
                    continuous = true
                }
                else if ((distance > DISTANCE_THRESHOLD) || (abs(distance - lastDistance)>0.5)) {
                    continuous = false
                }
                Log.i("color", hw.color.getDistance(DistanceUnit.CM).toString())
                if (gamepadState.rb1 || count >= 4){
                    state = State.OFF
                }
                if (gamepadState.lb1){
                    state = State.OUTTAKE
                }
                lastDistance = distance
            }
            State.OUTTAKE -> {
                val distance = hw.color.getDistance(DistanceUnit.CM)
                if (justTransitioned) {
                    hw.setIntakePower(OUTTAKE_POWER)
                }
                if(distance < DISTANCE_THRESHOLD && !continuous) {
                    count--
                    continuous = true
                }
                else if ((distance > DISTANCE_THRESHOLD) || (abs(distance - lastDistance)>0.5)) {
                    continuous = false
                }
                if (getElapsedSec(stateStartTimeNs) > 0.1){
                    state = State.OFF
                }
            }
            State.PRE_SHOOT -> {
                if (justTransitioned){
                    if (hw.pollenBallStop.position != POLLEN_GO) {
                        hw.pollenBallStop.position = POLLEN_GO
                    }
                    count = 0
                }
                if (getElapsedSec(stateStartTimeNs) > 0.1)
                    state = State.SHOOT
            }
            State.SHOOT -> {
                if (justTransitioned){
                    hw.setIntakePower(SHOOT_POWER)
                }
                if (getElapsedSec(stateStartTimeNs) > 1){
                    state = State.OFF
                }
            }

        }
    }
    private fun getElapsedSec(startTimeNs: Long): Double {
        return (System.nanoTime() - startTimeNs) / 1_000_000_000.0
    }

    private fun clearTimers() {
        val now = System.nanoTime()
        conditionContiniousStartTimeNs = 0L
        stateStartTimeNs = now
    }
}