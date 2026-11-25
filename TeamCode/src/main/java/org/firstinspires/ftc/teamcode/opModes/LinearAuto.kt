package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

import android.util.Log
import com.qualcomm.hardware.lynx.LynxModule
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
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.StopCondition
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.StopConditions
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils.StaticStore
import kotlin.math.hypot

abstract class LinearAuto(private val isRed: Boolean) : LinearOpMode() {

    lateinit var hw: GerthaHardwareMap

    enum class State{
        DRIVE_TO_SCORE,
        DRIVE_TO_LEFT_SCORE,
        SCAN_HIVE,
        SHOOT,
        DRIVE_TO_DRIVER_FLOWER,
        PICKUP_DRIVER_FLOWER,
        DRIVE_TO_OTHER_FLOWER,
        PICKUP_OTHER_FLOWER,
        WAYPOINT_OUT_OTHER_FLOWER,
        DRIVE_TO_GARDEN,
        PICKUP_GARDEN,
        HIVE_WAYPOINT_1,
        HIVE_WAYPOINT_2,
        PARK_RIGHT,
        PARK_LEFT,
        WAIT_FOR_RIGHT_TIP,
        WAIT_FOR_LEFT_TIP,
        WAIT_FOR_TIME,
        WAIT_UNDER_HIVE,
        PREPARK,
        DONE
    }

    open val autoSequence = listOf(
        State.DRIVE_TO_SCORE,
        State.SHOOT,
        State.DRIVE_TO_DRIVER_FLOWER,
        State.PICKUP_DRIVER_FLOWER,
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.DRIVE_TO_OTHER_FLOWER,
        State.PICKUP_OTHER_FLOWER,
        State.DRIVE_TO_OTHER_FLOWER,
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.DRIVE_TO_GARDEN,
        State.PICKUP_GARDEN,
        State.DRIVE_TO_SCORE,
        State.SCAN_HIVE,
        State.SHOOT,
        State.PREPARK,
        State.PARK_LEFT
    )

    private var currentIndex = 0

    lateinit var state: State
    var prevState = State.DONE

    var justTransitioned = true

    lateinit var robotState: RobotState

    private var conditionContiniousStartTimeNs: Long = 0L
    private var stateStartTimeNs: Long = 0L

    private var autoStartTimeNs: Long = 0L

    private var emptyLoops = 0

    var poseSet = PoseSet(isRed)
    open var startPose: RobotPose = poseSet.CLOSE_START


    override fun runOpMode() {

        state = autoSequence[currentIndex]

        hw = GerthaHardwareMap(hardwareMap)

        robotState = RobotState(hw, isRed, true)
        robotState.init(startPose)

        val gamepadState = GamepadState(gamepad1, gamepad2)

        //create state machines
        val shooterImpl = ShooterImpl(hw)
        val driveMachine = DriveMachine(hw)
        val intakeShootMachine = IntakeShootMachine(hw, isAuto = true)
        val rampMachine = RampMachine(hw)
        val aimingMachine = AimingMachine(hw, poseSet)
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
            telemetry.addData("ppx", robotState.ppX)
            telemetry.addData("ppy", robotState.ppY)
            telemetry.addData("ppa", robotState.ppThetaDeg)
            telemetry.update()
        }

        waitForStart()
        aimingMachine.state = AimingMachine.State.AUTO
        aimingMachine.setHardcode(AUTO_SHOOT_SPEED)
        driveMachine.state = DriveMachine.State.PID2POINT
        driveMachine.removerImpl.init(
            robotState.ppRobotPose,
            robotState.ppRobotPose
        )

        autoStartTimeNs = System.nanoTime()




        while (opModeIsActive()) {

            if (getElapsedSec(autoStartTimeNs) >= 27.0){
                state = State.PREPARK
            }

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
                    robotState.liveUpdate = true
                    if (justTransitioned){
                        if (robotState.rightHive){
                            if (distance(poseSet.SCORE_RIGHT_B) > 80.0) {
                                state = State.HIVE_WAYPOINT_1
                                continue
                            }
                            Log.i("driveToScore", "driving to right side when hive " + robotState.rightHive.toString())
                            driveMachine.removerImpl.init(
                                robotState.ppRobotPose,
                                poseSet.SCORE_RIGHT_B,
                                stopCond = StopConditions.Roughly
                            )
                        }
                        else {
                            if(distance(poseSet.SCORE_LEFT_B) > 80.0) {
                                state = State.HIVE_WAYPOINT_1
                                continue
                            }
                            Log.i("driveToScore", "driving to left side when hive " + robotState.rightHive.toString())
                            driveMachine.removerImpl.init(
                                robotState.ppRobotPose,
                                poseSet.SCORE_LEFT_B,
                                stopCond =  StopConditions.Roughly
                            )
                        }
                    }
                    if ((getElapsedSec(stateStartTimeNs) > 0.5) && (rampMachine.state != RampMachine.State.UP)){
                        rampMachine.state = RampMachine.State.UP
                    }
                    if (driveMachine.removerImpl.finished){
                        robotState.liveUpdate = false
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
                        if (distance(poseSet.DRIVER_FLOWER_PRESCORE) > 50) {
                            state = State.HIVE_WAYPOINT_1
                        }
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.DRIVER_FLOWER_PRESCORE,
                            stopCond = StopConditions.Roughly
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
                    robotState.liveUpdate = true
                    if (justTransitioned){
                        if (distance(poseSet.OTHER_FLOWER_PRESCORE) > 80.0) {
                            state = State.HIVE_WAYPOINT_1
                            continue
                        }
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.OTHER_FLOWER_PRESCORE,
                            stopCond = StopConditions.Roughly
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
                        robotState.liveUpdate = false
                    }
                }
                State.WAYPOINT_OUT_OTHER_FLOWER -> {
                    if (justTransitioned) {
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.OTHER_FLOWER_OUT,
                            stopCond = StopConditions.Waypoint
                        )
                    }

                    if(driveMachine.removerImpl.finished) {
                        rampMachine.state = RampMachine.State.UP
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
                State.HIVE_WAYPOINT_1 -> {
                    if (justTransitioned){
                        if (distance(poseSet.HIVE_WAYPOINT_RIGHT) < distance(poseSet.HIVE_WAYPOINT_LEFT)){
                            driveMachine.removerImpl.init(
                                robotState.ppRobotPose,
                                RobotPose(poseSet.HIVE_WAYPOINT_RIGHT.x, poseSet.HIVE_WAYPOINT_RIGHT.y, robotState.ppThetaRad),
                                stopCond = StopConditions.Waypoint
                            )
                        } else {
                            driveMachine.removerImpl.init(
                                robotState.ppRobotPose,
                                RobotPose(poseSet.HIVE_WAYPOINT_LEFT.x, poseSet.HIVE_WAYPOINT_LEFT.y, robotState.ppThetaRad),
                                stopCond = StopConditions.Waypoint
                            )
                        }
                    }
                    if ((getElapsedSec(stateStartTimeNs) > 0.5) && (rampMachine.state != RampMachine.State.UP)){
                        rampMachine.state = RampMachine.State.UP
                    }
                    if (driveMachine.removerImpl.finished){
                        if (autoSequence[currentIndex] == State.DRIVE_TO_DRIVER_FLOWER) {
                            state = autoSequence[currentIndex]
                            continue
                        }
                        state = State.HIVE_WAYPOINT_2
                    }
                }
                State.HIVE_WAYPOINT_2 -> {
                    if (justTransitioned) {
                        if (distance(poseSet.HIVE_WAYPOINT_RIGHT) < distance(poseSet.HIVE_WAYPOINT_LEFT)){
                            driveMachine.removerImpl.init(
                                robotState.ppRobotPose,
                                poseSet.HIVE_WAYPOINT_LEFT,
                                stopCond = StopConditions.Waypoint
                            )
                        } else {
                            driveMachine.removerImpl.init(
                                robotState.ppRobotPose,
                                poseSet.HIVE_WAYPOINT_RIGHT,
                                stopCond = StopConditions.Waypoint
                            )
                        }
                    }
                    if ((getElapsedSec(stateStartTimeNs) > 0.5) && (rampMachine.state != RampMachine.State.UP)){
                        rampMachine.state = RampMachine.State.UP
                    }
                    if (driveMachine.removerImpl.finished){
                        if (autoSequence[currentIndex] == State.SCAN_HIVE){
                            state = State.DRIVE_TO_SCORE
                        } else {
                            state = autoSequence[currentIndex]
                        }

                    }
                }
                State.PARK_RIGHT -> {
                    if (justTransitioned) {
                        rampMachine.state = RampMachine.State.UP
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.PARK_RIGHT
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        state = State.DONE
                    }
                }
                State.PARK_LEFT -> {
                    if (justTransitioned) {
                        rampMachine.state = RampMachine.State.UP
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.PARK_LEFT
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        state = State.DONE
                    }
                }
                State.DONE -> {
                    StaticStore.mark()
                }
                State.SCAN_HIVE -> {
                    if (justTransitioned) {
                        emptyLoops = 0
                    }
                    if (!robotState.isNotEmpty || robotState.onlySeesOtherColor) {
                        emptyLoops += 1
                    } else {
                        emptyLoops = 0
                    }

                    if (getElapsedSec(stateStartTimeNs) > 0.2) {
                        if (emptyLoops >=2 ){
                            robotState.rightHive = !robotState.rightHive
                            Log.i("auto", "flipped")
                        }
                        state = State.DRIVE_TO_SCORE
                    }
                }
                State.WAIT_FOR_RIGHT_TIP -> {
                    if (justTransitioned){
                        robotState.liveUpdate = true
                    }
                    if(robotState.rightHive) {
                        if (conditionContiniousStartTimeNs == 0L){
                            conditionContiniousStartTimeNs = System.nanoTime()
                        } else if (getElapsedSec(conditionContiniousStartTimeNs) > 0.4){
                            nextState()
                        }
                    } else {
                        conditionContiniousStartTimeNs = 0L
                    }
                }
                State.WAIT_FOR_LEFT_TIP -> {
                    if (justTransitioned){
                        robotState.liveUpdate = true
                    }
                    if(!robotState.rightHive) {
                        if (conditionContiniousStartTimeNs == 0L){
                            conditionContiniousStartTimeNs = System.nanoTime()
                        } else if (getElapsedSec(conditionContiniousStartTimeNs) > 0.4){
                            nextState()
                        }
                    } else {
                        conditionContiniousStartTimeNs = 0L
                    }
                }
                State.WAIT_FOR_TIME -> {
                    if (getElapsedSec(stateStartTimeNs) >= 15.0){
                        nextState()
                    }

                }
                State.DRIVE_TO_LEFT_SCORE -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.SCORE_LEFT_B
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.WAIT_UNDER_HIVE -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.UNDER_HIVE
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        nextState()
                    }
                }
                State.PREPARK -> {
                    if (justTransitioned){
                        driveMachine.removerImpl.init(
                            robotState.ppRobotPose,
                            poseSet.PARK_WAYPOINT,
                            stopCond = StopConditions.Waypoint
                        )
                    }
                    if (driveMachine.removerImpl.finished){
                        state = autoSequence[autoSequence.size -1]
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
        StaticStore.mark()
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