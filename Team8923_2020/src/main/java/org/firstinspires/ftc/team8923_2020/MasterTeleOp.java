package org.firstinspires.ftc.team8923_2020;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp (name="MasterTeleOp")
public abstract class MasterTeleOp extends MasterOpMode {
    public void driveMecanum() {

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double power = gamepad1.left_stick_y;
        double turnPower = gamepad1.right_stick_x;
        double angle = Math.atan2(-x, y);


        driveMecanum(angle,turnPower,power);
    }

    public void runIntake(){
        if(gamepad1.right_bumper){

        }

    }

    public void runLauncher(){

    }

    public void runWobbleGrabber(){

    }
}