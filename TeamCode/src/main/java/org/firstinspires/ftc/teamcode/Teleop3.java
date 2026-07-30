package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp()
public class Teleop3 extends LinearOpMode {
    ActionClass action;

    @Override
    public void runOpMode() {
        Init();

        waitForStart();
        if (opModeIsActive()) {
            // Pre-run

            while (opModeIsActive()) {
                // OpMode loop
                Intake();
                telemetry.update();
            }
        }
    }

    void Init(){
        action = new ActionClass(hardwareMap);
    }

    void Intake(){
        if (gamepad1.a){
            action.Intake_On();
        } else if (gamepad1.b) {
            action.Intake_R();
        }else{
            action.Intake_Off();
        }

    }

}
