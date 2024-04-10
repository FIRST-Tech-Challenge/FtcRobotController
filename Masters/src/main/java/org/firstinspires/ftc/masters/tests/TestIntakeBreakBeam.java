package org.firstinspires.ftc.masters.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.masters.drive.SampleMecanumDrive;

@TeleOp(name = "Sensor: Generic Switch", group = "Sensor")
//@Disabled
public class TestIntakeBreakBeam extends LinearOpMode {

    /*
     * Main loop
     */

    SampleMecanumDrive drive=null;
    @Override
    public void runOpMode() throws InterruptedException {

        /*
         * Initialize the hardware
         */

        drive = new SampleMecanumDrive(hardwareMap, telemetry);



        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            // is it on or off?

            if (gamepad1.a){
                drive.startIntake();
            }

            if (gamepad1.b){
                drive.stopIntake();
            }

            if (has2Pixels()){
                drive.stopIntake();
            }


        }
    }

    protected boolean has2Pixels(){
        return drive.frontBreakBeam.getState() && drive.backBreakBeam.getState();
    }
}

