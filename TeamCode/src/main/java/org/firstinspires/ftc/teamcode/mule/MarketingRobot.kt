package org.firstinspires.ftc.teamcode.mule

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime

@TeleOp
class MarketingRobot : OpMode() {

    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor

    lateinit var arm: DcMotor

    private var motionTimer = ElapsedTime()

    fun driveForward() {
        fl.power = 0.1
        fr.power = 0.1
        bl.power = 0.1
        br.power = 0.1
    }

    fun driveBackwards() {
        fl.power = -0.1
        fr.power = -0.1
        bl.power = -0.1
        br.power = -0.1
    }

    fun armLeft() {
        arm.power = 0.1
    }

    fun armRight() {
        arm.power = -0.1
    }

    override fun init() {
        fl = hardwareMap.get(DcMotor::class.java, "FL")
        fr = hardwareMap.get(DcMotor::class.java, "FR")
        bl = hardwareMap.get(DcMotor::class.java, "BL")
        br = hardwareMap.get(DcMotor::class.java, "BR")
        arm = hardwareMap.dcMotor["Arm"]

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        if (motionTimer.seconds() < 2.0) {
            driveForward()
            armLeft()
        }
        if (motionTimer.seconds() > 2.0) {
            driveBackwards()
            armRight()
        }
        if (motionTimer.seconds() > 4.0) {
            motionTimer.reset()
        }
    }
}