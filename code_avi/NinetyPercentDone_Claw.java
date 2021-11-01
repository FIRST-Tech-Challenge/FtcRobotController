package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Ninety Claw", group="TeleOp")
public class NinetyPercentDone_Claw extends OpMode {

    //creating linear slide objects
    LinearSlide armController;
    LinearSlide clawController;

    //init function
    @Override
    public void init() {
        //telemetry a
        telemetry.addData("Hello! Initializing!", "웃");
        telemetry.update();

        //get motors from hardware map
        DcMotor arm = hardwareMap.dcMotor.get("arm");
        DcMotor claw = hardwareMap.dcMotor.get("claw");

        //declare linear slide controllers
        armController = new LinearSlide(arm, 0,360);
        clawController = new LinearSlide(claw, 0,360);

        //telemetry b
        telemetry.addData("Ready for launch!" , "＼(≧▽≦)／");
        telemetry.addData("WARNING!" , "LINEAR SLIDE IS OPERATING IN UNRESTRICTED MODE");
        telemetry.update();
    }

    //loop function
    @Override
    public void loop() {

        //arm code
        armController.MoveSlideUnrestricted(gamepad2.left_stick_y);

        //claw speed setter
        float clawSpeed = 0;
        if(gamepad2.y){
            clawSpeed = 0.5f;
        }
        else if(gamepad2.a){
            clawSpeed = -0.5f;
        }

        //moves claw
        clawController.MoveSlideUnrestricted(clawSpeed);


    }
}
