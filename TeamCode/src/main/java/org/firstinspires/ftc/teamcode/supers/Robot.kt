package org.firstinspires.ftc.teamcode.supers

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.SwerveModule
import org.firstinspires.ftc.teamcode.util.GamepadState

// Kotlin is a stupid language made by stupid people, used by stupid people and i hate it and it's not as compatible with java as the feds want you to think.
class Robot (private var opMode: OpMode) {
    // Declare all the hardware
    var lt: DcMotor
    var lb: DcMotor
    var rt: DcMotor
    var rb: DcMotor
    var sl: SwerveModule
    var sr: SwerveModule
    private var motors: Array<DcMotor>

    // Declare gamepads
    var gamepadState1: GamepadState = GamepadState()
    var gamepadState2: GamepadState = GamepadState()

    // Set opMode and hardwareMap
    init {
        val hardwareMap: HardwareMap = opMode.hardwareMap

        // Get all the motors
        lt = hardwareMap.get(DcMotor::class.java, "lf")
        lb = hardwareMap.get(DcMotor::class.java, "lb")
        rt = hardwareMap.get(DcMotor::class.java, "rf")
        rb = hardwareMap.get(DcMotor::class.java, "rb")

        motors = arrayOf(lt, lb, rt, rb)

        sl = SwerveModule(lt, lb, 14.35, 0.0)
        sr = SwerveModule(rt, rb, -14.35, 0.0)

        // Make sure all the motors are in what should be the default
        for (motor in motors) {
            motor.power = 0.0
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        }
    }
}

// noooow ieeee knowl dendude nahenune nAaah melatha