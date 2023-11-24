package org.firstinspires.ftc.teamcode.Teleop

//import org.firstinspires.ftc.teamcode.Variables.rMotorL
//import org.firstinspires.ftc.teamcode.Variables.rMotorR
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR
import java.io.BufferedWriter
import java.io.FileWriter

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
            telemetry.addLine("Length of Commands: "+commandList.size);
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
            sleep(50);
            commandList.add("sleep:50");
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
