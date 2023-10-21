
package org.firstinspires.ftc.teamcode.Teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.DriveMethods

@TeleOp(name = "ClawControl", group = "AprilTag")
class ClawControl: DriveMethods() {
    var servo1: Servo? = null;
    var servo2: Servo? = null;


    override fun runOpMode() {
        //turn
        servo1 = hardwareMap.get<Servo>(Servo::class.java, "servo1");
        //claw thing
        servo2 = hardwareMap.get<Servo>(Servo::class.java, "servo2");
        telemetry.addData("Initiating", "Servo Motors Init");
        telemetry.update()
        waitForStart()
        var changed1 = false;
        var changed2= false;
        servo1!!.position = 0.3
        servo2!!.position = 0.871
        while (opModeIsActive()) {
          telemetry.addLine(" Thang: "+servo1!!.position+" Thangy: "+servo2!!.position)
            telemetry.update()
          if(gamepad1.dpad_down&&!changed1&& servo1!!.position>0.21){
              servo1!!.position-=0.02;
              changed1=true;
          }else if(gamepad1.dpad_up&&!changed1&&servo1!!.position<0.57){
                servo1!!.position+=0.02;
              changed1=true;
          }else if(!gamepad1.dpad_down&&!gamepad1.dpad_up){
              changed1=false;
          }

          if(gamepad1.dpad_left&&!changed2&&servo2!!.position>0.6){
            servo2!!.position-=0.01;
              changed2=true;
          }else if(gamepad1.dpad_right&&!changed2&&servo2!!.position<0.87){
            servo2!!.position+=0.01;
              changed2=true;
          }else if(!gamepad1.dpad_left&&!gamepad1.dpad_right){
              changed2=false;
          }


        sleep(10);
        }
    }
}
