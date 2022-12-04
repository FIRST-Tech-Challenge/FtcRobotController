package org.firstinspires.ftc.teamcode.powerplay.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.powerplay.Robot
import org.firstinspires.ftc.teamcode.util.math.MathUtil

@TeleOp
class TestOp: OpMode() {
//    private val robot = Robot()

    lateinit var fl: DcMotor
    lateinit var fr: DcMotor
    lateinit var bl: DcMotor
    lateinit var br: DcMotor

    lateinit var slides : DcMotor

    var drive = 0.0
    var strafe = 0.0
    var rotate = 0.0

    private fun driveControl() {
        drive = MathUtil.cubicScaling(0.75, -gamepad1.left_stick_y.toDouble()) * 0.85
        strafe = MathUtil.cubicScaling(0.75, gamepad1.left_stick_x.toDouble()) * 0.85
        rotate = gamepad1.right_stick_x.toDouble() * 0.65
        fl.power = drive + strafe + rotate
        fr.power = drive - strafe - rotate
        bl.power = drive - strafe + rotate
        br.power = drive + strafe - rotate
    }

//    private fun clawControl() {
//        if (gamepad1.a) {
//            robot.clawOpened()
//        }
//        if(gamepad1.b) {
//            robot.clawClosed()
//        }
//    }
//
    private fun slidesControl() {
        if (gamepad1.dpad_up) {
            slides.power = 1.0
        }
        else {
            slides.power = 0.0
        }

        if (gamepad1.dpad_down) {
            slides.power = 0.4
        }
        else {
            slides.power = 0.0
        }
    }
//
//    private fun getTelemetry() {
//    }

    override fun init() {
//        robot.init(hardwareMap)
//        robot.reset()

        fl = hardwareMap.get(DcMotor::class.java, "fl")
        fr = hardwareMap.get(DcMotor::class.java, "fr")
        bl = hardwareMap.get(DcMotor::class.java, "bl")
        br = hardwareMap.get(DcMotor::class.java, "br")

        slides = hardwareMap.get(DcMotor::class.java, "Lift")

        fl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        fr.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        bl.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        br.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        fl.direction = DcMotorSimple.Direction.REVERSE
        bl.direction = DcMotorSimple.Direction.REVERSE
    }

    override fun loop() {
        driveControl()
//        clawControl()
        slidesControl()
//        getTelemetry()
//        robot.update()
    }
}