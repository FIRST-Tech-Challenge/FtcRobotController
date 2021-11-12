package org.firstinspires.ftc.teamcode.mechanism

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.Range

class KotlinLift: Mechanism {
    var targetPosition = 0.0
    lateinit var liftMotor: DcMotorEx
    override fun init(hardwareMap: HardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx::class.java, "lift")
        liftMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
    }

    override fun run(gamepad: Gamepad) {
        targetPosition += gamepad.left_stick_y
        targetPosition = Range.clip(targetPosition, 0.0, 5000.0)
        liftMotor.targetPosition = targetPosition.toInt()
        liftMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        liftMotor.power = .5
    }
}