package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables
import org.firstinspires.ftc.teamcode.Variables.bottom
import org.firstinspires.ftc.teamcode.Variables.clawMotor
import org.firstinspires.ftc.teamcode.Variables.clawRotation
import org.firstinspires.ftc.teamcode.Variables.click2Degree
import org.firstinspires.ftc.teamcode.Variables.closedClaw
import org.firstinspires.ftc.teamcode.Variables.high
import org.firstinspires.ftc.teamcode.Variables.lMax
import org.firstinspires.ftc.teamcode.Variables.lMin
import org.firstinspires.ftc.teamcode.Variables.lPower
import org.firstinspires.ftc.teamcode.Variables.lPowerSlow
import org.firstinspires.ftc.teamcode.Variables.lSpeedMax
import org.firstinspires.ftc.teamcode.Variables.lSpeedMin
import org.firstinspires.ftc.teamcode.Variables.length
import org.firstinspires.ftc.teamcode.Variables.low
import org.firstinspires.ftc.teamcode.Variables.mid
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR
import org.firstinspires.ftc.teamcode.Variables.motorSlideLeft
import org.firstinspires.ftc.teamcode.Variables.openClaw
import org.firstinspires.ftc.teamcode.Variables.rMax
import org.firstinspires.ftc.teamcode.Variables.rMin
//import org.firstinspires.ftc.teamcode.Variables.rMotorL
//import org.firstinspires.ftc.teamcode.Variables.rMotorR
import org.firstinspires.ftc.teamcode.Variables.rPower
import org.firstinspires.ftc.teamcode.Variables.rPowerSlow
import org.firstinspires.ftc.teamcode.Variables.rSpeedMax
import org.firstinspires.ftc.teamcode.Variables.rSpeedMin
import org.firstinspires.ftc.teamcode.Variables.slideGate
import org.firstinspires.ftc.teamcode.Variables.slideRotMax
import org.firstinspires.ftc.teamcode.Variables.slideRotMin
import org.firstinspires.ftc.teamcode.Variables.slideRotationMotor
import org.firstinspires.ftc.teamcode.Variables.speed
import org.firstinspires.ftc.teamcode.Variables.t
import java.io.BufferedWriter
import java.io.FileWriter
import kotlin.math.abs
import kotlin.math.exp

@TeleOp(name = "SimpleTeleopRecorder", group = "TeleopFinal")
class SimpleTeleopRecorder: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondSimple() //init rack and pinion & wheel motors

        telemetry.update()


        waitForStart()
        var leftY: Double
        var leftX: Double
        var rightX: Double
        var upOrDown = true
        var speedDiv = 3.0
        var commandList = ArrayList<String>();

        while (opModeIsActive()) {
            //set gamepad inputs
            leftY = (-gamepad1.left_stick_y).toDouble()
            leftX = -gamepad1.left_stick_x.toDouble()
            rightX = -gamepad1.right_stick_x.toDouble()

            //set motor speeds
            val motorFLPower = (leftY - leftX - rightX) / speedDiv
            val motorBLPower = (leftY + leftX - rightX) / speedDiv
            val motorFRPower = (leftY + leftX + rightX) / speedDiv
            val motorBRPower = (leftY - leftX + rightX) / speedDiv

            telemetry.addLine("Power of motorFL: $motorFLPower")
            telemetry.addLine("Power of motorFR: $motorFRPower")
            telemetry.addLine("Power of motorBL: $motorBLPower")
            telemetry.addLine("Power of motorBR: $motorBRPower")

            setMotorPowers(motorFLPower, motorFRPower, motorBLPower, motorBRPower)
            commandList.add("motorFL.power:$motorFLPower");
            commandList.add("motorFR.power:$motorFRPower");
            commandList.add("motorBL.power:$motorBLPower");
            commandList.add("motorBR.power:$motorBRPower");

            if (gamepad1.left_bumper) {
                motorFL?.power = 1.0/speedDiv
                motorBL?.power = -1.0/speedDiv
                motorFR?.power = 1.0/speedDiv
                motorBR?.power = -1.0/speedDiv

                commandList.add("motorFL.power:"+(1.0/speedDiv))
                commandList.add("motorBL.power:"+(-1.0/speedDiv))
                commandList.add("motorFR.power:"+(1.0/speedDiv))
                commandList.add("motorBR.power:"+(-1.0/speedDiv))
            }

            if (gamepad1.right_bumper) {
                motorFL?.power = -1.0/speedDiv
                motorBL?.power = 1.0/speedDiv
                motorFR?.power = -1.0/speedDiv
                motorBR?.power = 1.0/speedDiv

                commandList.add("motorFL.power:"+(-1.0/speedDiv))
                commandList.add("motorBL.power:"+(1.0/speedDiv))
                commandList.add("motorFR.power:"+(-1.0/speedDiv))
                commandList.add("motorBR.power:"+(1.0/speedDiv))
            }

            if (gamepad1.a) {
                if (speedDiv == 3.0) {
                    speedDiv = 1.5
                } else {
                    speedDiv = 3.0
                }
            }

            telemetry.update()

        }

        var outputWriter: BufferedWriter? = null
        outputWriter = BufferedWriter(FileWriter("/sdcard/FIRST/training/SimpleRecordTest.txt"));
        for (i in 0 .. commandList.size-1) {
            outputWriter.write(commandList[i]);
            outputWriter.newLine();
        }
        outputWriter.flush();
        outputWriter.close();
    }
}
