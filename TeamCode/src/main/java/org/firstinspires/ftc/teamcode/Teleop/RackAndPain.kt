
package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.Variables.rMotorL
import org.firstinspires.ftc.teamcode.Variables.rMotorR
import org.firstinspires.ftc.teamcode.Variables.touchyL
import org.firstinspires.ftc.teamcode.Variables.touchyR

@TeleOp(name = "RackAndPain", group = "AprilTag")
class RackAndPain: DriveMethods() {
    override fun runOpMode() {
        initMotorsSecondBot()

        var lClick =0;
        var rClick = 0;
        var lPower = 0.2
        var rPower = 0.22280685416
        var leftRack = 1;
        var rightRack = -1;
        telemetry.addData("Initiating", "Rack Motor Init");
        telemetry.update()

        waitForStart()
        var mode = 0;
        //0 = increase, 1 = decrease;
        var increaseMode = 0;
        var changed_mode = false
        var changed_mode2 = false;
        //mode 0 = both, 1 = left, 2 = right
        while (opModeIsActive()) {
            var ending = false;
            if (touchyR!!.isPressed) {
                rMotorR!!.setPower(.4*rightRack)
                telemetry.addLine("R Sensor Detected")
                ending = true;
            }
            if (touchyL!!.isPressed){
                rMotorL!!.setPower(.4*leftRack)
                telemetry.addLine("L Sensor Detected")
                ending = true;
            }
            if (ending){
              continue;
            }

            lClick = (rMotorL!!.getCurrentPosition())
            rClick = (rMotorR!!.getCurrentPosition())
            telemetry.addLine("Current LPos: "+lClick+"\nCurrent RPos: "+rClick);
            telemetry.addLine("LPower: " + lPower + "\nRPower: " + rPower);
            telemetry.addLine("Mode: "+mode)
            telemetry.addLine("IncreaseMode: "+increaseMode)
            telemetry.addLine("Left touch sensor pressed is: " + touchyL!!.isPressed)
            telemetry.addLine("Right touch sensor pressed is: " + touchyR!!.isPressed)
            telemetry.addLine()
            telemetry.update()

            if (!changed_mode&&gamepad1.y) {
                mode++;
                if (mode > 2) {
                    mode = 0;
                }
                changed_mode = true;
            }
            if(changed_mode&&!gamepad1.y) {
                changed_mode = false;
            }


            if (!changed_mode2&&gamepad1.x){
                if (increaseMode<1){
                    increaseMode+=1;
                }else {
                    increaseMode=0;
                }
                changed_mode2=true;
            }
            if(changed_mode2&&!gamepad1.x){
                changed_mode2=false;
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
                if (mode == 0) {
                    rMotorR!!.power = rPower * rightRack;
                    rMotorL!!.power = lPower * leftRack;
                } else if (mode == 1) {
                    rMotorL!!.power = lPower * leftRack;
                } else if (mode == 2) {
                    rMotorR!!.power = rPower * rightRack;
                }
            }

            else if(gamepad1.dpad_down) {
                if (mode == 0) {
                    rMotorR!!.power = -rPower * rightRack;
                    rMotorL!!.power = -lPower * leftRack;
                } else if (mode == 1) {
                    rMotorL!!.power = -lPower * leftRack;
                } else if (mode == 2) {
                    rMotorR!!.power = -rPower * rightRack;
                }
            }
            else{
                rMotorR!!.power = 0.0;
                rMotorL!!.power = 0.0;
            }
            sleep(10)
        }
    }
}
