package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
public class Practice extends OpMode {
    @Override
    public void init(){
        telemetry.addData("Initiated", "True");
    }

    @Override
    public void loop(){

        double leftStickX = gamepad1.left_stick_x;
        double leftStickY = -gamepad1.left_stick_y;
        double motorPower = gamepad1.left_trigger;
        boolean touchSense_1 = false;
        String ID = "2920jaj";

        if(motorPower >= 0){
            telemetry.addLine("Positive Motor Power");
        }

        else{
            telemetry.addLine("Negative Motor Power");
        }

        // Using an AND statement
        if((leftStickX >= 0) && (leftStickY >= 0)){
            telemetry.addLine("Moving in a positive direction.");
        }

       /* while(gamepad1.a){
        **IMPORTANT** This does NOT work. The Gamepad only updates in between loop(). The variable MUST update inside the loop, or else it will infinitely repeat.
        }
        */

        if(!gamepad1.a){
            motorPower /= 2;
        }

        else{
            motorPower /= 1;
        }


    }
}
