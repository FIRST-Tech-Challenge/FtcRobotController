
package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.rMotorL
import org.firstinspires.ftc.teamcode.Variables.rMotorR
import org.firstinspires.ftc.teamcode.Variables.touchyL
import org.firstinspires.ftc.teamcode.Variables.touchyR

@Disabled
@TeleOp(name = "RackAndPain", group = "AprilTag")
class RackAndPain: DriveMethods() {
    override fun runOpMode() {
        initOnlyRackAndPain()

        var lClick: Int
        var rClick: Int
        var lPower = 0.2
        var rPower = 0.22280685416
        val leftRack = 1
        val rightRack = -1
        telemetry.addData("Initiating", "Rack Motor Init");
        telemetry.update()

        waitForStart()
        var mode = 0
        //0 = increase, 1 = decrease;
        var increaseMode = 0
        var changedMode = false
        var changedMode2 = false
        //mode 0 = both, 1 = left, 2 = right
        while (opModeIsActive()) {
            var ending = false
            if (touchyR!!.isPressed) {
                rMotorR!!.power = .4*rightRack
                telemetry.addLine("R Sensor Detected")
                ending = true;
            }
            if (touchyL!!.isPressed){
                rMotorL!!.power = .4*leftRack
                telemetry.addLine("L Sensor Detected")
                ending = true;
            }
            if (ending){
              continue;
            }

            lClick = (rMotorL!!.currentPosition)
            rClick = (rMotorR!!.currentPosition)
            telemetry.addLine("Current LPos: $lClick\nCurrent RPos: $rClick")
            telemetry.addLine("LPower: $lPower\nRPower: $rPower")
            telemetry.addLine("Mode: $mode")
            telemetry.addLine("IncreaseMode: $increaseMode")
            telemetry.addLine("Left touch sensor pressed is: " + touchyL!!.isPressed)
            telemetry.addLine("Right touch sensor pressed is: " + touchyR!!.isPressed)
            telemetry.addLine()
            telemetry.update()

            if (!changedMode&&gamepad1.y) {
                mode++
                if (mode > 2) {
                    mode = 0
                }
                changedMode = true
            }
            if(changedMode&&!gamepad1.y) {
                changedMode = false
            }


            if (!changedMode2&&gamepad1.x){
                if (increaseMode<1){
                    increaseMode+=1
                }else {
                    increaseMode=0
                }
                changedMode2=true
            }
            if(changedMode2&&!gamepad1.x){
                changedMode2=false
            }


            if(gamepad1.a) {
                if(increaseMode==0) {
                    lPower += .01
                } else if (increaseMode==1) {
                    lPower -= 0.01
                }
            }
            if(gamepad1.b) {
                if(increaseMode==0) {
                    rPower += .01
                } else if (increaseMode==1){
                    rPower -= 0.01
                }
            }

            if (gamepad1.dpad_up) {
                when {
                    (mode == 0) -> {
                        rMotorR!!.power = rPower * rightRack
                        rMotorL!!.power = lPower * leftRack
                    }
                    (mode == 1) -> {
                        rMotorL!!.power = lPower * leftRack
                    }
                    (mode == 2) -> {
                        rMotorR!!.power = rPower * rightRack
                    }
                }
            }

            else if(gamepad1.dpad_down) {
                if (mode == 0) {
                    rMotorR!!.power = -rPower * rightRack
                    rMotorL!!.power = -lPower * leftRack
                } else if (mode == 1) {
                    rMotorL!!.power = -lPower * leftRack
                } else if (mode == 2) {
                    rMotorR!!.power = -rPower * rightRack
                }
            }
            else{
                rMotorR!!.power = 0.0
                rMotorL!!.power = 0.0
            }
            sleep(10)
        }
    }
}
