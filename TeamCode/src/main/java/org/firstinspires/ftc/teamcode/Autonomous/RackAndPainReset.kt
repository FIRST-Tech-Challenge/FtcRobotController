package org.firstinspires.ftc.teamcode.Autonomous

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.DriveMethods

@Autonomous(name = "RackAndPainReset", group = "AprilTag")
class RackAndPainReset: DriveMethods() {
    var rMotorR: DcMotor? = null;
    var rMotorL: DcMotor? = null;
    var touchyR: TouchSensor? = null
    var touchyL: TouchSensor? = null


    override fun runOpMode() {
        //init
//        initMotorsSecondBot()
        rMotorR = hardwareMap.get<DcMotor>(DcMotor::class.java, "rMotorR");
        rMotorL = hardwareMap.get<DcMotor>(DcMotor::class.java, "rMotorL");
        touchyR = hardwareMap.get<TouchSensor>(TouchSensor::class.java, "touchyR")
        touchyL = hardwareMap.get<TouchSensor>(TouchSensor::class.java, "touchyL")
        var lClick =0;
        var rClick = 0;
        var lPower = 0.2
        var rPower = 0.2 * 1.26923076923 * 0.87772397094
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
        var endR = false;
        var endL = false;
        while (opModeIsActive()) {
            var ending = false;
            if (touchyR!!.isPressed) {
                rMotorR!!.setPower(.4*rightRack)
                telemetry.addLine("R Sensor Detected")
                endR=true;
                ending = true;
                rMotorR!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                rMotorR!!.mode = DcMotor.RunMode.RUN_USING_ENCODER;
            }else if(!endR){
            rMotorR!!.setPower(-0.4*rightRack)
            }
            if (touchyL!!.isPressed){
                rMotorL!!.setPower(.4*leftRack)
                telemetry.addLine("L Sensor Detected")
                endL=true;
                ending = true;
                rMotorL!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                rMotorL!!.mode = DcMotor.RunMode.RUN_USING_ENCODER;
            }else if(!endL){
            rMotorL!!.setPower(-0.4*leftRack)
            }
            
            if(endR){
            rMotorR!!.setPower(0.0)

            }
            if(endL){
            rMotorL!!.setPower(0.0)
            }

            if (ending){
              continue;
            }
            
          sleep(10)
        }
    }
}
