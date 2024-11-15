package org.firstinspires.ftc.teamcode.helpers

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorControllerEx
import com.qualcomm.robotcore.hardware.DcMotorEx
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.RADIANS

/**
 * defines the angular velocity of a DcMotorEx in AngleUnits per second.
 * Maximal spindle velocity is 200π Radians per second or 36,000 degrees per second
 */
//inline var DcMotorEx.angularVelocity: MotorVelocity
//    get() {
//        return MotorVelocity(this.getVelocity(RADIANS))
//    }
//    set(value) {
//        this.setVelocity(value.value, RADIANS)
//    }
//
//inline val DcMotor.controllerEx: DcMotorControllerEx
//    get() {
//        return this.controller as DcMotorControllerEx
//    }