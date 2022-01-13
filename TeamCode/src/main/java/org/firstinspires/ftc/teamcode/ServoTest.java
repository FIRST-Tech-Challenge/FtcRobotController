package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name="servo", group="Testier")
public class ServoTest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    int servopos = 0;
//    boolean isServoOn = false;
//    boolean isServoOff = true;
    boolean isPrimed = false;
    private double waitTime;
    Servos reeceBox = new Servos(); // Output servo - Reece's box

    public void init(){
        reeceBox.init(hardwareMap);
//        reeceBox.changePos(0.25);
    }
    public void loop(){
//        waitTime = .125;
//        time = runtime.seconds();

//        if(gamepad1.y&&servopos<2){
//            servopos++;
//            runtime.reset();
//            while (waitTime > runtime.time());
//        }
//        else if(gamepad1.y){
//            servopos = 0;
//        }

        // Toggles servo pess on
//        if (gamepad1.y && !isServoOn) {
//            isServoOn = true;
////            if(servopos<2){
////                servopos++;
////            }
////            else {
////                servopos=0;
////            }
//        }else if (!gamepad1.y && isServoOn) {
//            isServoOff = false;
//        }
//
//        //Turn servo press off
//        if (gamepad1.y && !isServoOff) {
//            isServoOn = false;
//        }else if (!gamepad1.y && !isServoOn) {
//            isServoOff = true;
        telemetry.addData("isPrimed",isPrimed);
        telemetry.addData("Position",servopos);
        telemetry.update();
        if(gamepad1.y){

            isPrimed = true;
        }
        else if(isPrimed && !gamepad1.y){
            if(servopos<2){
               servopos++;
            }
            else {
               servopos=0;
            }
            isPrimed = false;
        }
        else {
            isPrimed = false;
        }

        switch (servopos){
            case 0:
                reeceBox.changePos(0.3);
                break;
            case 1:
                reeceBox.changePos(.5);
                break;
            case 2:
                reeceBox.changePos(.92);
                break;
        }

    }
}
