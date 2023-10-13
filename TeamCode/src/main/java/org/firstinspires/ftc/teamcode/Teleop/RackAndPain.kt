
package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import org.firstinspires.ftc.teamcode.DriveMethods

@TeleOp(name = "RackAndPain", group = "AprilTag")
class RackAndPain: DriveMethods() {
    var rMotorR: DcMotor? = null;
    var rMotorL: DcMotor? = null;


    override fun runOpMode() {
        //init
//        initMotorsSecondBot()
        rMotorR = hardwareMap.get<DcMotor>(DcMotor::class.java, "rMotorR");
        rMotorL = hardwareMap.get<DcMotor>(DcMotor::class.java, "rMotorL");
        var lClick =0;
        var rClick = 0;
        var lPower = 0.2
        var rPower = 0.2
        telemetry.addData("Initiating", "Rack Motor Init");
        telemetry.update()

        waitForStart()
        
        while (opModeIsActive()) {
            lClick = (rMotorL!!.getCurrentPosition())
            rClick = (rMotorR!!.getCurrentPosition())
            telemetry.addLine("Current LPos: "+lClick+"\nCurrent RPos: "+rClick);
            telemetry.addLine("LPower: " + lPower + "\nRPower: " + rPower);
            telemetry.addLine()
            telemetry.update()
            var leftRack = 1;
            var rightRack = -1;
            if(gamepad1.a) {
                lPower += .01
            }
            if(gamepad1.b) {
                rPower += .01
            }
            if(gamepad1.dpad_left) {
                rMotorL!!.setPower(lPower*leftRack)
            }
            if(gamepad1.dpad_right) {
                rMotorR!!.setPower(rPower*rightRack)
            }
            if (gamepad1.dpad_up){
              rMotorR!!.setPower(rPower*rightRack);
              rMotorL!!.setPower(lPower*leftRack);
            }
            else if(gamepad1.dpad_down){
              rMotorR!!.setPower(-rPower*rightRack);
              rMotorL!!.setPower(-lPower*leftRack);
            }
            else{
              rMotorR!!.setPower(0.0);
              rMotorL!!.setPower(0.0);
            }
            sleep(10)
        }
    }
}
