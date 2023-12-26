package org.firstinspires.ftc.teamcode.botmodule

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Servo

/**
 * Intake controls
 */
@Suppress("unused")
class Intake(opMode: OpMode, private val servoLift: Servo?, private val motorSpin: DcMotorEx?) : BotModule(opMode) {

    private var intakePos = 0;
    //pos
    //1 - pizel off ground
    //1-5 - pixel off stack
    //6 starting pos within 18 inches

    init {
        setPos(intakePos)
    }

    // 0.0 is off, 1.0 is inwards, -1.0 is outwards

    fun spin(power: Double){
        motorSpin?.power = power
    }

    fun lower() {
        if (intakePos > 1){
            intakePos--
            setPos(intakePos)
        }
    }

    fun raise() {
        if (intakePos <6){
            intakePos ++
            setPos(intakePos)
        }
    }

    public fun setPos(pos: Int){
        intakePos = pos
        when(pos){
            1 -> servoLift?.position = 1.0//These numbers are placeholders and will be changed later
            2 -> servoLift?.position = 1.0
            3 -> servoLift?.position = 1.0
            4 -> servoLift?.position = 1.0
            5 -> servoLift?.position = 1.0
            6 -> servoLift?.position = 1.0 // This should be all the way up
            else -> return


        }
    }
}
