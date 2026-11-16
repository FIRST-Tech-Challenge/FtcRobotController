package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.hardware.RobotPose
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.hardware.PoseSet
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.DriveMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.GamepadState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.IntakeShootMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RampMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RobotState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.StopConditions
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils.StaticStore
import kotlin.math.hypot

@Autonomous
class LinearAuto : LinearOpMode() {

    lateinit var hw: GerthaHardwareMap

    enum class State{
        DRIVE_TO_SCORE,
        SCAN_HIVE,
        SHOOT,
        DRIVE_TO_DRIVER_FLOWER,
        PICKUP_DRIVER_FLOWER,
        DRIVE_TO_OTHER_FLOWER,
        PICKUP_OTHER_FLOWER,
        DRIVE_TO_GARDEN,
        PICKUP_GARDEN,
        HIVE_WAYPOINT,
        PARK_RIGHT,
        PARK_LEFT,
        DONE
    }

    private val autoSequence = listOf(
        State.HIVE_WAYPOINT,
        State.DRIVE_TO_SCORE,
        State.HIVE_WAYPOINT,
        State.DRIVE_TO_OTHER_FLOWER,
        State.PICKUP_OTHER_FLOWER,
        State.HIVE_WAYPOINT,
        State.DRIVE_TO_SCORE,
        State.DRIVE_TO_DRIVER_FLOWER,
        State.PICKUP_DRIVER_FLOWER,
        State.DRIVE_TO_SCORE,
        State.DRIVE_TO_GARDEN,
        State.PICKUP_GARDEN,
        State.DRIVE_TO_SCORE,
        State.PARK_RIGHT,
        State.DONE
    )

    private var currentIndex = 0

    private var state = autoSequence[currentIndex]
    var prevState = state

    var justTransitioned = true

    lateinit var robotState: RobotState

    private var conditionContiniousStartTimeNs: Long = 0L
    private var stateStartTimeNs: Long = 0L

    override fun runOpMode() {

        hw = GerthaHardwareMap(hardwareMap)

        var poseSet = PoseSet()

        robotState = RobotState(hw)
        robotState.init(poseSet.CORNER_START)

        val gamepadState = GamepadState(gamepad1, gamepad2)

        //create state machines
        val driveMachine = DriveMachine(hw)
        val intakeShootMachine = IntakeShootMachine(hw)
        val rampMachine = RampMachine(hw)
        //aim

        //prep to bulk read
        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        driveMachine.init(robotState)
        intakeShootMachine.init()
        rampMachine.init()

        //remover roll to starting pose
        while(opModeInInit()){
            robotState.update()
        }

        waitForStart()

        driveMachine.state = DriveMachine.State.PID2POINT
        driveMachine.removerImpl.init(
            robotState.ppRobotPose,
            poseSet.SCORE_RIGHT
        )


        while (opModeIsActive()) {

            for (module in lynxModules) {
                module.clearBulkCache()
            }
            robotState.update()
            gamepadState.update() //need for auto??

            justTransitioned = (state != prevState)
            prevState = state

            if (justTransitioned){
                clearTimers()
            }

            when (state) {
                State.DRIVE_TO_SCORE -> {
                    if (justTransitioned){
                        if (poseSet.RIGHT_HIVE){
                            driveMachine.removerImpl.init(
                                robotState.ppRobotPose,
                                poseSet.SCORE_RIGHT
                            )
                        }
                        else {
                            driveMachine.removerImpl.init(
                                robotState.ppRobotPose,
                                poseSet.SCORE_LEFT
                            )
                        }
                    }
                    if ((getElapsedSec(stateStartTimeNs) > 0.5) && (rampMachine.state != RampMachine.State.UP)){
                        rampMachine.state = RampMachine.State.UP
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.SHOOT -> {
                    if (justTransitioned){
                        intakeShootMachine.state = IntakeShootMachine.State.PRE_SHOOT
                    }
                    if (intakeShootMachine.state == IntakeShootMachine.State.OFF){
                        nextState()
                    }
                }
                State.DRIVE_TO_DRIVER_FLOWER -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.DRIVER_FLOWER_PRESCORE
                        )
                    }
                    if (distance(poseSet.DRIVER_FLOWER_PRESCORE) < 12.0){
                        rampMachine.state = RampMachine.State.FLOWER_INTAKE
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.PICKUP_DRIVER_FLOWER -> {
                    if (justTransitioned){
                        intakeShootMachine.state = IntakeShootMachine.State.ON
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.DRIVER_FLOWER_INTAKE
                        )
                    }
                    if (intakeShootMachine.state == IntakeShootMachine.State.OFF || getElapsedSec(stateStartTimeNs) > 3.0) {
                        nextState()
                    }
                }
                State.DRIVE_TO_OTHER_FLOWER -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.OTHER_FLOWER_PRESCORE
                        )
                    }
                    if (distance(poseSet.OTHER_FLOWER_PRESCORE) < 12.0){
                        rampMachine.state = RampMachine.State.FLOWER_INTAKE
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.PICKUP_OTHER_FLOWER -> {
                    if (justTransitioned){
                        intakeShootMachine.state = IntakeShootMachine.State.ON
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.OTHER_FLOWER_INTAKE
                        )
                    }
                    if (intakeShootMachine.state == IntakeShootMachine.State.OFF || getElapsedSec(stateStartTimeNs) > 3.0) {
                        nextState()
                    }
                }
                State.DRIVE_TO_GARDEN -> {
                    if (justTransitioned) {
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.PREGARDEN
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.PICKUP_GARDEN -> {
                    if (justTransitioned){
                        intakeShootMachine.state = IntakeShootMachine.State.ON
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.GARDEN
                        )
                    }
                    if (intakeShootMachine.state == IntakeShootMachine.State.OFF || getElapsedSec(stateStartTimeNs) > 3.0) {
                        nextState()
                    }
                }
                State.HIVE_WAYPOINT -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.HIVE_WAYPOINT,
                            stopCond = StopConditions.Waypoint
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.PARK_RIGHT -> {
                    if (justTransitioned) {
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.PARK_RIGHT
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.PARK_LEFT -> {
                    if (justTransitioned) {
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.PARK_LEFT
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.DONE -> {
                    StaticStore.mark()
                }
                State.SCAN_HIVE -> {
                    //
                }
            }

            driveMachine.update(robotState, gamepadState)
            intakeShootMachine.update(robotState, gamepadState)
            rampMachine.update(gamepadState)

            telemetry.addData("targetx", driveMachine.removerImpl.target.x)
            telemetry.addData("targety", driveMachine.removerImpl.target.y)
            telemetry.addData("currentX", robotState.ppX)
            telemetry.addData("currentY", robotState.ppY)
            telemetry.addData("currentA", robotState.ppThetaDeg)
            telemetry.update()

            Log.i("state", state.toString())
            Log.i("removerImpl", driveMachine.removerImpl.finished.toString())

        }
    }
    fun nextState(){
        currentIndex++
        state = autoSequence.getOrElse(currentIndex) {State.DONE}
    }

    fun distance(point: RobotPose): Double {
        val dx = robotState.ppX - point.x
        val dy = robotState.ppY - point.y

        val dist = hypot(dx, dy)

        return (dist)
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