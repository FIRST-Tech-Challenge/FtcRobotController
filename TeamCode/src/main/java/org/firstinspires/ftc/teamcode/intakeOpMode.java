package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class intakeOpMode extends OpMode {

    intake intake = new intake();

    @Override
    public void init() {
        intake.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.b){
            intake.setMotorSpeed(1.0);
        }
        else{
            intake.setMotorSpeed(0.0);
        }
    }
}
