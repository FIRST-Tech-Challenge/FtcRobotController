
package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.bottom
import org.firstinspires.ftc.teamcode.Variables.lMax
import org.firstinspires.ftc.teamcode.Variables.lMin
import org.firstinspires.ftc.teamcode.Variables.lPower
import org.firstinspires.ftc.teamcode.Variables.lPowerSlow
import org.firstinspires.ftc.teamcode.Variables.lSpeedMax
import org.firstinspires.ftc.teamcode.Variables.lSpeedMin
import org.firstinspires.ftc.teamcode.Variables.motorBL
import org.firstinspires.ftc.teamcode.Variables.motorBR
import org.firstinspires.ftc.teamcode.Variables.motorFL
import org.firstinspires.ftc.teamcode.Variables.motorFR
import org.firstinspires.ftc.teamcode.Variables.rMax
import org.firstinspires.ftc.teamcode.Variables.rMin
import org.firstinspires.ftc.teamcode.Variables.rMotorL
import org.firstinspires.ftc.teamcode.Variables.rMotorR
import org.firstinspires.ftc.teamcode.Variables.rPower
import org.firstinspires.ftc.teamcode.Variables.rPowerSlow
import org.firstinspires.ftc.teamcode.Variables.rSpeedMax
import org.firstinspires.ftc.teamcode.Variables.rSpeedMin
import java.io.BufferedWriter
import java.io.FileWriter


@TeleOp(name = "TeleopFromTheDownLow", group = "TeleopFinal")
class TeleopFromTheDownLow: DriveMethods() {
    override fun runOpMode() {
        //perfectly record everything at the exact perfectness that is needed

        initMotorsSecondBot() //init rack and pinion & wheel motors
        initSlideMotors() //init claw/slide motors

        when ((0..31).random()) {
            1 -> telemetry.addLine("good luck buddy")
            2 -> telemetry.addLine("\"what spectrum?\"")
            3 -> telemetry.addLine("MostlyOp >>> AHoT")
            4 -> telemetry.addLine("01101011 01101001 01101100 01101100 00100000 01111001 01101111 01110101 01110010 01110011 01100101 01101100 01100110")
            5 -> telemetry.addLine("I LOVE ULTRAKILL!!!!!!!!!!!!")
            6 -> telemetry.addLine("\"just hit clean build\"")
            7 -> telemetry.addLine("this match is gonna be ghoulish green")
            8 -> telemetry.addLine("we are so back")
            9 -> telemetry.addLine("ok so would you rather have a 1% chance of becoming a turkey everyday or...")
            10 -> telemetry.addLine("RIP damien mode 2022-2023")
            11 -> telemetry.addLine("build freeze at 3 AM challenge (GONE WRONG)")
            12 -> telemetry.addLine("\"who unqueued my song?\"")
            13 -> telemetry.addLine("at least we don't have a pushbot! (not confirmed, high likelyhood of pushbot)")
            14 -> telemetry.addLine("whoever set continuous rotation as the default is my #1 opp")
            15 -> telemetry.addLine("shoutout to Huy for being our insider <3")
            16 -> telemetry.addLine("why does jack always come to TR3? Is he stupid?")
            17 -> telemetry.addLine("Nick, I need you to sand this.")
            18 -> telemetry.addLine("I wish fame and good fortune upon Sachals bloodline")
            19 -> telemetry.addLine("-((2 / (1 + (exp(-(target - Pos) / speed)))) - 1) * max")
            20 -> telemetry.addLine("\"the grid system is stupid.\" *starts pointing at poles*")
            21 -> telemetry.addLine("James, how many orange cups have you eaten today?")
            22 -> telemetry.addLine("Tennisball is the newest sport sweeping across the nation!")
            23 -> telemetry.addLine("our robot has been too big for the bounding box on 3 different occasions.")
            24 -> telemetry.addLine("cord control is not real")
            25 -> telemetry.addLine("in Raytheon we trust")
            26 -> telemetry.addLine("drive practice is for nerds.")
            27 -> telemetry.addLine("Sebastian (yum)")
            28 -> telemetry.addLine("this is the abyss of our hero's journey.")
            29 -> telemetry.addLine("beware the FTC to Raytheon pipeline")
            30 -> telemetry.addLine("when build says 15 minutes, expect 30. When programming says 15 minutes, expect 2-60.")
            31 -> telemetry.addLine("99% of programmers quit right before the working push")
            32 -> telemetry.addLine("Tiger Woods PGA tour 2005 has always been there")
        }
        telemetry.update()
        waitForStart()
        var leftY: Double
        var leftYGPadTwo: Double
        var leftX: Double
        var rightX: Double
        var placeOrCollect = true
        var upOrDown = true
        var slideTarget = bottom
        var targetHeight = 0
        var slidePos = 0
        var slideRotPos = 0
        var speedDiv = 3.0
        var slideDeg = 0.0
        var angleFromSlideToClaw = 0.0
        var slideRottarget = 25.0
        var clawClamp = false
        var commandList = ArrayList<String>();

        while (opModeIsActive()) {
            //set gamepad inputs
            leftY = (-gamepad1.left_stick_y).toDouble()
            leftYGPadTwo = (gamepad2.left_stick_y).toDouble()
            leftX = -gamepad1.left_stick_x.toDouble()
            rightX = -gamepad1.right_stick_x.toDouble()

            //set motor speeds
            motorFL?.power = -(leftY - leftX - rightX) / speedDiv
            commandList.add("motorFl.power:"+(-(leftY - leftX - rightX) / speedDiv));
            motorBL?.power = -(leftY + leftX - rightX) / speedDiv
            commandList.add("motorBL.power:"+(-(leftY + leftX - rightX) / speedDiv));
            motorFR?.power = (leftY + leftX + rightX) / speedDiv
            commandList.add("motorFR.power:"+((leftY + leftX + rightX) / speedDiv));
            motorBR?.power = (leftY - leftX + rightX) / speedDiv
            commandList.add("motorBR.power:"+((leftY - leftX + rightX) / speedDiv));

            //open/close claw
            if (gamepad2.b) {
                //clawMotor!!.position = closedClaw
            }
            if (gamepad2.a) {
                //clawMotor!!.position = openClaw
            }

            if (gamepad1.left_bumper) {
                motorFL?.power = 1.0/speedDiv
                motorBL?.power = -1.0/speedDiv
                motorFR?.power = 1.0/speedDiv
                motorBR?.power = -1.0/speedDiv
                commandList.add("motorFL.power:"+(1.0/speedDiv));
                commandList.add("motorBL.power:"+(-1.0/speedDiv));
                commandList.add("motorFR.power:"+(1.0/speedDiv));
                commandList.add("motorBR.power:"+(-1.0/speedDiv));
            }

            if (gamepad1.right_bumper) {
                motorFL?.power = -1.0/speedDiv
                motorBL?.power = 1.0/speedDiv
                motorFR?.power = -1.0/speedDiv
                motorBR?.power = 1.0/speedDiv

                commandList.add("motorFL.power:"+(-1.0/speedDiv));
                commandList.add("motorBL.power:"+(1.0/speedDiv));
                commandList.add("motorFR.power:"+(-1.0/speedDiv));
                commandList.add("motorBR.power:"+(1.0/speedDiv));

            }

            if (gamepad1.a) {
                if (speedDiv == 3.0) {
                    speedDiv = 1.5
                } else {
                    speedDiv = 3.0
                }
            }

            //rack & pinion control
            if (gamepad2.right_bumper) {
                if (upOrDown) {
                    if (rMotorL?.currentPosition!! <= lMax) {
                        if (rMotorL?.currentPosition!! in lSpeedMin..lSpeedMax) {
                            telemetry.addLine("LUp")
                            rMotorL!!.power = lPower
                            commandList.add("rMotorL.power:"+(lPower));
                        } else {
                            rMotorL!!.power = lPowerSlow
                            commandList.add("rMotorL.power"+(lPowerSlow));
                        }
                    }
                } else {
                    if (rMotorL?.currentPosition!! >= lMin) {
                        if (rMotorL?.currentPosition!! in lSpeedMin..lSpeedMax) {
                            telemetry.addLine("LUp")
                            rMotorL!!.power = -lPower
                            commandList.add("rMotorL.power:"+(-lPower));
                        } else {
                            rMotorL!!.power = -lPowerSlow
                            commandList.add("rMotorL.power"+(-lPowerSlow));
                        }
                    }
                }
            } else {
                rMotorL!!.power = 0.0
                commandList.add("rMotorL.power:"+(0.0));
            }
            if (gamepad2.right_trigger >= 0.5) {
               // telemetry.addLine("edrgthgj")
                if (upOrDown) {
                    if (rMotorR?.currentPosition!! >= rMax) {
                        if (rMotorR?.currentPosition!! in rSpeedMax..rSpeedMin) {
                            rMotorR!!.power = rPower
                            commandList.add("rMotorR.power:"+(rPower));
                        } else {
                            rMotorR!!.power = rPowerSlow
                            commandList.add("rMotorR.power:"+(rPowerSlow));
                        }
                        telemetry.addLine("RMAX")
                    }
                } else {
                    if (rMotorR?.currentPosition!! <= rMin) {
                        if (rMotorR?.currentPosition!! in rSpeedMax..rSpeedMin) {
                            rMotorR!!.power = -rPower
                            commandList.add("rMotorR.power:"+(-rPower));
                        } else {
                            rMotorR!!.power = -rPowerSlow
                            commandList.add("rMotorR.power:"+(-rPowerSlow));
                        }
                        telemetry.addLine("RMIN")
                    }
                }
            } else {
                rMotorR!!.power = 0.0
                commandList.add("rMotorR.power:"+(0.0));
            }
            if (gamepad2.x) {
                telemetry.addData("upOrDown", upOrDown)
                upOrDown = !upOrDown
                sleep(500)
                commandList.add("sleep:500");
            }
            telemetry.update()


//            All Stuff for Temp Passive intake claw

            var passiveServo = hardwareMap.get(Servo::class.java, "passiveServo")
            if(gamepad2.a) {
                if (clawClamp) {
                    clawClamp = !clawClamp
                    passiveServo.position = 0.72;
                    commandList.add("passiveServo.position:"+(0.72));
                }
                else {
                    clawClamp = !clawClamp
                    passiveServo.position = .5
                    commandList.add("passiveServo.position:"+(0.5));
                }
                sleep(200)
                commandList.add("sleep:200");
            }
            telemetry.addData("Claw Clamped: ", clawClamp)
            telemetry.addData("Right rack: ", rMotorR?.currentPosition)
            telemetry.addData("Left rack: ", rMotorL?.currentPosition)
            telemetry.update()
        }
        var outputWriter: BufferedWriter? = null
        outputWriter = BufferedWriter(FileWriter("/sdcard/FIRST/training/test.txt"));
        for (i in 0..<commandList.size) {
            outputWriter.write(commandList[i]);
            outputWriter.newLine();
        }
        outputWriter.flush();
        outputWriter.close();

    }
}
