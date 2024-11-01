package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.util.Range.clip;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.TemplateJanx;

@TeleOp(name = "what will show up on driver hub")
public class Template extends LinearOpMode {
    //replace Template with your class name
    //initiate motors
    // private motorType nameOfMotor
    @Override
    public void runOpMode() {
        TemplateJanx janx = new TemplateJanx(hardwareMap);
        //link motors to config
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                //call functions here
            }
        }
    }
    //write functions here
}