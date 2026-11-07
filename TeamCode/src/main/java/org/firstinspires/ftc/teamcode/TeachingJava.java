package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;



@Disabled
@TeleOp
public class TeachingJava extends OpMode {


    private double kevin;

    @Override
    public void init() {
        kevin = 1;

    }

    @Override
    public void loop() {
        method();
        if(kevin == 2){
            try {
                Thread.sleep(5000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
    }
    private void method(){
        telemetry.addData("hi", kevin);
        telemetry.update();
        kevin = kevin + 1;
    }
}
