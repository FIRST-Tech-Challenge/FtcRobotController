package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems

import android.util.Log
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.*
import org.firstinspires.ftc.teamcode.ShooterImpl
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

        if (robotState.blue - robotState.green > 100 && robotState.blue - robotState.red > 100) {
            Log.i("color", "blue")
            state = State.OUTTAKE
        }
        else if (robotState.red - robotState.blue > 100 && robotState.red - robotState.green > 100) {
            Log.i("color", "red")
            state = State.OUTTAKE
        }

        when (state) {
            State.OFF -> {
                if (justTransitioned) {
                    hw.setIntakePower(0.0)
                }
                if (gamepadState.rb1){
                    state = State.ON
                } else if (gamepadState.lb1) {
                    state = State.OUTTAKE
                } else if (gamepadState.b2 || gamepadState.b1) {
                    state = State.PRE_SHOOT
                }
            }
            State.ON -> {
                val distance = hw.color.getDistance(DistanceUnit.CM)
                if (justTransitioned) {
                    hw.pollenBallStop.position = POLLEN_STOP
                    hw.setIntakePower(INTAKE_POWER)
                }
                if(distance < DISTANCE_THRESHOLD) {
                   if (conditionContiniousStartTimeNs == 0L){
                       conditionContiniousStartTimeNs = System.nanoTime()
                   } else if (getElapsedSec(conditionContiniousStartTimeNs) > 0.5){
                       state = State.OFF
                   }
                } else {
                    conditionContiniousStartTimeNs = 0L
                }

                Log.i("color", hw.color.getDistance(DistanceUnit.CM).toString())
                if (gamepadState.rb1){
                    state = State.OFF
                }
                if (gamepadState.lb1){
                    state = State.OUTTAKE
                }
            }
            State.OUTTAKE -> {
                if (justTransitioned) {
                    hw.setIntakePower(OUTTAKE_POWER)
                }
                if (getElapsedSec(stateStartTimeNs) > 0.25){
                    state = State.OFF
                }
            }
            State.PRE_SHOOT -> {
                if (justTransitioned){
                    if (hw.pollenBallStop.position != POLLEN_GO) {
                        hw.pollenBallStop.position = POLLEN_GO
                    }
                }
                if (getElapsedSec(stateStartTimeNs) > 0.0)
                    state = State.SHOOT
            }
            State.SHOOT -> {
                if (justTransitioned){
                    hw.setIntakePower(SHOOT_POWER)
                }
                if (getElapsedSec(stateStartTimeNs) > 2.0){
                    state = State.OFF
                }
            }

        }

        Log.i("intake", state.toString())
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