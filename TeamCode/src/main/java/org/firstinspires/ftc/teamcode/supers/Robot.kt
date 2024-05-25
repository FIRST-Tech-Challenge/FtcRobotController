package org.firstinspires.ftc.teamcode.supers

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.util.GamepadState

// Kotlin is a stupid language made by stupid people, used by stupid people and i hate it and it's not as compatible with java as the feds want you to think.
class Robot (opMode: OpMode) {
    // Declare all the hardware
    private lateinit var lf: DcMotor
    private lateinit var lb: DcMotor
    private lateinit var rf: DcMotor
    private lateinit var rb: DcMotor
    private val motors: Array<DcMotor> = arrayOf(lf, lb, rf, rb)

    // Declare gamepads
    var gamepadState1: GamepadState = GamepadState()
    var gamepadState2: GamepadState = GamepadState()

    private var opMode: OpMode

    init {
        // Set opMode and hardwareMap
        this.opMode = opMode
        val hardwareMap: HardwareMap = opMode.hardwareMap

        // Get all the motors
        lf = hardwareMap.get(DcMotor::class.java, "lf")
        lb = hardwareMap.get(DcMotor::class.java, "lb")
        rf = hardwareMap.get(DcMotor::class.java, "rf")
        rb = hardwareMap.get(DcMotor::class.java, "rb")

        // Make sure all the motors are in what should be the default
        for (motor in motors) {
            motor.power = 0.0
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }
}

// noooow ieeee knowl dendude nahenune nahbleuvenAaah melatha