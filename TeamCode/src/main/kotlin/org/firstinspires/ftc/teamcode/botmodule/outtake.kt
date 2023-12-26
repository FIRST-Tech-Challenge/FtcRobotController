package org.firstinspires.ftc.teamcode.botmodule

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm
import com.qualcomm.robotcore.hardware.PIDFCoefficients
import com.qualcomm.robotcore.hardware.Servo

/**
 * Linear Slide Driver
 */
class outtake(opMode: OpMode, private val rightSlide: DcMotorEx, private val leftSlide: DcMotorEx, private val rightArm: Servo?, private val leftArm: Servo?, private val rightClaw: Servo?, private val leftClaw: Servo?) : BotModule(opMode) {
    private var placeRow = 0;



    //11 rows

    companion object {

        @JvmStatic private val RIGHT_ARM_PLACEMENT_POS: Double = 1.0//Placeholders
        @JvmStatic private val LEFT_ARM_PLACEMENT_POS: Double = -1.0
        @JvmStatic private val RIGHT_ARM_PICKUP_POS: Double = 1.0
        @JvmStatic private val LEFT_ARM_PICKUP_POS: Double = -1.0
        @JvmStatic private val RIGHT_CLAW_CLOSE_POS: Double = -1.0
        @JvmStatic private val LEFT_CLAW_CLOSE_POS: Double = 1.0
    }
    fun closeClaws(){
        rightClaw?.position = RIGHT_CLAW_CLOSE_POS
        leftClaw?.position = LEFT_CLAW_CLOSE_POS
    }

    fun openLeftClaw(){
        leftClaw?.position = 0.0
    }

    fun openRightClaw(){
        rightClaw?.position = 0.0
    }


    fun increaseRow(){
        if(placeRow <11){
            placeRow++
        }
    }

    fun decreaseRow(){
        if (placeRow > 0){
            placeRow--
        }
    }

    fun resetRow(){
        placeRow = 0
    }

    fun runToRow(){
        closeClaws()
        rightArm?.position = RIGHT_ARM_PLACEMENT_POS
        leftArm?.position = LEFT_ARM_PLACEMENT_POS
        when(placeRow){
            0 -> runToPickup()

            else -> return
        }
    }

    fun runToPickup(){
        rightArm?.position = RIGHT_ARM_PICKUP_POS
        leftArm?.position = LEFT_ARM_PICKUP_POS

        rightSlide.targetPosition = 0
        leftSlide.targetPosition = 0

        rightSlide.mode = RUN_TO_POSITION
        leftSlide.mode = RUN_TO_POSITION

        rightSlide.power = 0.7
        leftSlide.power = 0.7

        rightClaw?.position = 0.0
        leftClaw?.position = 0.0


    }

    fun place(){
        leftClaw?.position = 0.0
        rightClaw?.position = 0.0
        runToPickup()
    }


}