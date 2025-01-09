package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/* Robot include */
import org.firstinspires.ftc.teamcode.configurations.Configuration;

@TeleOp
public class ManualOpMode extends OpMode {

    Moving          moving;
    Collecting      collecting;

    @Override
    public void init(){

        moving = new Moving();
        collecting = new Collecting();
        
        moving.setHW(Configuration.s_Current, hardwareMap, telemetry, gamepad1);
        collecting.setHW(Configuration.s_Current, hardwareMap, telemetry, gamepad2);
        
    }
    public void loop (){
        
        moving.move();
        collecting.move();
        
    }
}
