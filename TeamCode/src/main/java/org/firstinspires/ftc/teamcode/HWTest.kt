package org.firstinspires.ftc.teamcode

import android.util.Log
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.hardware.Encoder
import kotlin.math.abs

@TeleOp
class HWTest : LinearOpMode() {
    private class StopException: RuntimeException()

    lateinit var hw: Hardware

    val results: MutableMap<String, Boolean> = mutableMapOf()

    var wasAPressed: Boolean = false
    var wasBPressed: Boolean = false

    fun stopAll() {
        hw.driveMotors.setAll(0.0)
    }

    fun clearCtrl() {
        while (gamepad1.a || gamepad1.b) {
            if (!opModeIsActive()) throw StopException()
            telemetry.addLine("Waiting for operator to release the controls...")
            telemetry.update()
        }
        wasAPressed = false
        wasBPressed = false
    }

    fun alert(message: String) {
        telemetry.addLine(message)
        telemetry.addLine()
        telemetry.addLine("Press A to continue.")
        telemetry.update()
        while (opModeIsActive()) {
            val a = gamepad1.a
            if (a && !wasAPressed) break
            wasAPressed = a
        }
        clearCtrl()
    }

    fun yesno(message: String, yes: String = "Yes", no: String = "No"): Boolean {
        telemetry.addLine(message)
        telemetry.addLine()
        telemetry.addLine("[A]: $yes. [B]: $no.")
        telemetry.update()
        var res = true
        while (opModeIsActive()) {
            val a = gamepad1.a
            val b = gamepad1.b
            if (a && !wasAPressed) {
                res = true
                break
            }
            if (b && !wasBPressed) {
                res = false
                break
            }
            wasAPressed = a
            wasBPressed = b
        }
        clearCtrl()
        return res
    }

    fun testMotor(motor: DcMotor, label: String) {
        motor.power = 0.2
        results[label] = yesno("Is $label moving forward?")
        motor.power = 0.0
    }

    fun testEncoderA(enc: Encoder, label: String) {
        val basis = enc.getCurrentPosition()
        var res = false
        while (opModeIsActive()) {
            telemetry.addLine("Move the $label...")
            telemetry.addLine("or press B to skip")
            telemetry.update()
            val delta = enc.getCurrentPosition() - basis
            if (abs(delta) > 10) {
                res = true
                break
            }
            val b = gamepad1.b
            if (b && !wasBPressed) {
                res = false
                break
            }
            wasBPressed = b
        }
        results[label] = res
        clearCtrl()
    }

    fun testEncoderB(enc: DcMotor, label: String) {
        val basis = enc.currentPosition
        var res = false
        while (opModeIsActive()) {
            telemetry.addLine("Move the $label...")
            telemetry.addLine("or press B to skip")
            telemetry.update()
            val delta = enc.currentPosition - basis
            if (abs(delta) > 10) {
                res = true
                break
            }
            val b = gamepad1.b
            if (b && !wasBPressed) {
                res = false
                break
            }
            wasBPressed = b
        }
        results[label] = res
        clearCtrl()
    }

    fun askContinue() {
        if (!yesno("Continue?", "Yes", "Abort")) throw StopException()
    }

    fun makeReport(): String = buildString {
        for ((k, v) in results) {
            append(k)
            append(": ")
            append(if (v) "yes" else "no")
            append("\n")
        }
    }

    override fun runOpMode() {
        hw = Hardware(hardwareMap)
        telemetry.addLine("Start the OpMode to continue.")
        telemetry.addLine("Ensure gamepad1 is connected.")
        telemetry.update()
        waitForStart()
        results.clear()
        try {
            stopAll()
            if (yesno("Test drive motors?")) {
                alert("Lift the robot so the wheels aren't touching the floor.")
                testMotor(hw.frontLeft, "frontLeft")
                testMotor(hw.frontRight, "frontRight")
                testMotor(hw.backRight, "backRight")
                testMotor(hw.backLeft, "backLeft")
                stopAll()
            }
            if (yesno("Test encoders?")) {
                testEncoderA(hw.encoderLeft, "encLeft")
                testEncoderA(hw.encoderCenter, "encCenter")
                testEncoderA(hw.encoderRight, "encRight")
                testEncoderA(hw.encoderVerticalSlide, "encVerticalSlide")
                testEncoderB(hw.arm, "encArm")
            }
            if (yesno("Test lift?")) {
//                hw.verticalLift.setTargetPosition(Hardware.VLIFT_SCORE_SPECIMEN)
//                while (opModeIsActive()) {
//
//                }
                stopAll()
            }
        } catch (_: StopException) {}
        Log.i("Inspection Report", makeReport())
        while (opModeIsActive()) {
            for (line in makeReport().split("\n")) {
                if (line.trim() == "") continue
                telemetry.addLine(line)
            }
            telemetry.update()
        }
    }
}