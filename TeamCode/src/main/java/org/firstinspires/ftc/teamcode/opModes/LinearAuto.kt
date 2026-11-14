package org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.opModes

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.robot.Robot
import org.firstinspires.ftc.teamcode.GerthaHardwareMap
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.DriveMachine
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.GamepadState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.subsystems.RobotState
import org.firstinspires.ftc.teamcode.org.firstinspires.ftc.teamcode.utils.StaticStore

class LinearAuto : LinearOpMode() {

    lateinit var hw: GerthaHardwareMap

    enum class State{
        DRIVE_FLOWER1,
        DONE
    }

    private val autoSequence = listOf(
        State.DRIVE_FLOWER1
    )

    private var currentIndex = 0

    private var state = autoSequence[currentIndex]
    var prevState = state

    var justTransitioned = true

    lateinit var robotState: RobotState

    override fun runOpMode() {

        hw = GerthaHardwareMap(hardwareMap)

        robotState = RobotState(hw)
        //add something so this doesn't crash when you don't have both gamepads plugged in
        val gamepadState = GamepadState(gamepad1, gamepad2)

        //create state machines
        val driveMachine = DriveMachine(hw)
        //intakeShoot
        //aim

        //prep to bulk read
        val lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        //init everything
//        robotState.ppX //finish this
        driveMachine.init()

        //remover roll to starting pose
        while(opModeInInit()){
            robotState.update()
        }

        waitForStart()

        while (opModeIsActive()) {
            StaticStore.mark()

            for (module in lynxModules) {
                module.clearBulkCache()
            }
            robotState.update()
            gamepadState.update() //need for auto??

            justTransitioned = (state != prevState)
            prevState = state

            when (state) {
                State.DRIVE_FLOWER1 -> {
                    if (justTransitioned){
                        //hi
                    }
                    //hello
                }
                State.DONE -> {
                    StaticStore.mark()
                }
            }

        }
    }
    fun nextState(){
        currentIndex++
        state = autoSequence.getOrElse(currentIndex) {State.DONE}
    }
}