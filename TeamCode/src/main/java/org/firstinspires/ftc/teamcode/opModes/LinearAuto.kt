package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.GerthaHardwareMap.*
import org.firstinspires.ftc.teamcode.ShooterImpl
import org.firstinspires.ftc.teamcode.hardware.RobotPose
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.hardware.PoseSet
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.AimingMachine
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
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.DRIVE_TO_OTHER_FLOWER,
        State.PICKUP_OTHER_FLOWER,
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.DRIVE_TO_DRIVER_FLOWER,
        State.PICKUP_DRIVER_FLOWER,
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.DRIVE_TO_GARDEN,
        State.PICKUP_GARDEN,
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.PARK_RIGHT,
        State.DONE
    )

    private var currentIndex = 0

    private var state = autoSequence[currentIndex]
    var prevState = State.DONE

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
        val shooterImpl = ShooterImpl(hw)
        val driveMachine = DriveMachine(hw)
        val intakeShootMachine = IntakeShootMachine(hw)
        val rampMachine = RampMachine(hw)
        val aimingMachine = AimingMachine(hw)
        //aim

        //prep to bulk read
        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        driveMachine.init(robotState)
        intakeShootMachine.init()
        rampMachine.init()
        aimingMachine.init()

        //remover roll to starting pose
        while(opModeInInit()){
            robotState.update()
        }

        waitForStart()
        aimingMachine.state = AimingMachine.State.AUTO
        aimingMachine.setHardcode(AUTO_SHOOT_SPEED)
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
                        if (robotState.rightHive){
                            if (distance(poseSet.SCORE_RIGHT) > 80.0) {
                                state = State.HIVE_WAYPOINT
                                continue
                            }
                            Log.i("driveToScore", "driving to right side when hive " + robotState.rightHive.toString())
                            driveMachine.removerImpl.init(
                                robotState.ppRobotPose,
                                poseSet.SCORE_RIGHT
                            )
                        }
                        else {
                            if(distance(poseSet.SCORE_LEFT) > 100) {
                                state = State.HIVE_WAYPOINT
                                continue
                            }
                            Log.i("driveToScore", "driving to left side when hive " + robotState.rightHive.toString())
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
                        if (distance(poseSet.OTHER_FLOWER_PRESCORE) > 80.0) {
                            state = State.HIVE_WAYPOINT
                            continue
                        }
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
                            RobotPose(poseSet.HIVE_WAYPOINT.x, poseSet.HIVE_WAYPOINT.y, robotState.ppThetaRad),
                            stopCond = StopConditions.Waypoint
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        state = autoSequence[currentIndex]
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
                    if (justTransitioned) {

                    }

                    if (getElapsedSec(stateStartTimeNs) > 0.1) {
                        if (!robotState.isNotEmpty || robotState.onlySeesOtherColor) {
                            Log.i("empty", robotState.isNotEmpty.toString())
                            Log.i("onlyOtherColor", robotState.onlySeesOtherColor.toString())
                            Log.i("auto", "flipped")
                            robotState.rightHive = !robotState.rightHive
                        }
                        state = State.DRIVE_TO_SCORE
                    }
                }
            }

            driveMachine.update(robotState, gamepadState)
            intakeShootMachine.update(robotState, gamepadState)
            rampMachine.update(gamepadState)
            aimingMachine.update(robotState,gamepadState)

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