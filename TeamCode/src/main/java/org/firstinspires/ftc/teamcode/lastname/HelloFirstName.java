package org.firstinspires.ftc.teamcode.lastname;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class HelloFirstName extends OpMode {
    @Override
    public void init() {
        telemetry.addData("Hello","FirstName");
    }
    public void loop (){
        
    }
}
