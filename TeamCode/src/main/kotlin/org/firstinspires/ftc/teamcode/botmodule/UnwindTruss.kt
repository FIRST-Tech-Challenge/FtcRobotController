package org.firstinspires.ftc.teamcode.botmodule

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.BotShared

@TeleOp(name = "Unwind Truss")
class UnwindTruss(opMode: OpMode, private val motorLift: DcMotorEx?, private val hangArmRight: Servo?, private val hangArmLeft: Servo?) : BotModule(opMode) {

    private lateinit var shared: BotShared

    fun init() {

    }

    fun raiseArms() {
        hangArmLeft?.position = 0.1//placeholder
        hangArmRight?.position = 0.1//placeholder
    }

    fun lift(power: Double){
        motorLift?.power = power
    }

}