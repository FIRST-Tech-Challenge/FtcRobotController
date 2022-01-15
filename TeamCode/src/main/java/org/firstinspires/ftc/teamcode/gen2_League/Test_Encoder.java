package org.firstinspires.ftc.teamcode.gen2_League;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_Encoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor liftControl = hardwareMap.get(DcMotor.class,"liftControl");
        Servo hopper = hardwareMap.get(Servo.class,"hopper");


        int high = 3500;
        int mid = 1750;
        int low = 800;
        int ground = 0;

        waitForStart();
        if (opModeIsActive()) {

            int tickCount = 560;

            liftControl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            liftControl.setTargetPosition(low);
            liftControl.setPower(0.8);
            liftControl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if(liftControl.getCurrentPosition() >= low ) {
                hopper.setPosition(1.0);
            }


            while (opModeIsActive()) {

                telemetry.addData("Position", liftControl.getCurrentPosition());
                telemetry.update();
            }

        }


    }


}
