package org.firstinspires.ftc.teamcode;

/* Qualcomm includes */
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/* Robot include */
import org.firstinspires.ftc.teamcode.configurations.Configuration;

@TeleOp
public class ManualOpMode extends OpMode {

    Moving          moving;
    Claw            claw;

    @Override
    public void init(){

        moving = new Moving();
        claw = new Claw();
        moving.setHW(Configuration.s_Current, hardwareMap, telemetry, gamepad1);
        claw.setHW(Configuration.s_Current, hardwareMap, telemetry, gamepad2);
    }
    public void loop (){
        moving.move();
        claw.move();
    }
}
