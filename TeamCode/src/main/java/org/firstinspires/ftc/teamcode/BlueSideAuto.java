package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BlueSideAuto",group = "SkyStone")
public class BlueSideAuto extends BaseAuto {

    Drivetrain drivetrain;


    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware();

        initializeOpenCV();


        drivetrain = new Drivetrain(robot);


        //Wait for the start button to be pressed
        waitForStart();

        /*while (opModeIsActive())
        {

            double gyroangle = getAverageGyro();
            telemetry.addData("angle: ", gyroangle);
            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Position", pipeline.position);
            telemetry.update();

            // Don't burn CPU cycles busy-looping in this sample
            sleep(50);
        }*/

//        encoderMecanumDrive(0.3, 20, 3, 0, 1);
//        telemetry.addData("imu", getAverageGyro());
//        telemetry.update();
//        gyroTurn(0.3,90);
//        telemetry.addData("imu", getAverageGyro());
//        telemetry.update();

        sleep(1000);
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        sleep(1000);
        if (pipeline.position.toString() == "FOUR"){
            telemetry.addData("Going with four", "");
            telemetry.update();
            sleep(1000);
//            encoderMecanumDrive(0.3, 20, 3, 0, -1);
            encoderMecanumDrive(0.5, 185, 5, -0.464,-1);


        } else if (pipeline.position.toString() == "ONE"){
            telemetry.addData("Going with one", "");
            telemetry.update();
            sleep(1000);

        } else {
            telemetry.addData("Going with none", "");
            telemetry.update();
            sleep(1000);

        }
        drivetrain.tilt(0.35);
        sleep(300);
        gyroTurn(0.3, getAverageGyro()-20);
        sleep(1000);



//        encoderMecanumDrive(DRIVE_SPEED,50,4,-1,0);
//        encoderMecanumDrive(DRIVE_SPEED,50,4,0,-1);


//        telemetry.addData("location",location);
//        telemetry.update();

        /*if (location.equals("Right")){
            encoderMecanumDrive(DRIVE_SPEED,5,1,0,1);
            gyroTurn(TURN_SPEED,60);
            encoderMecanumDrive(DRIVE_SPEED,6,1,-0.8660254,-0.5);
            succ();
            encoderMecanumDrive(0.2,60,10,-0.5,0.8660254);
            succstop();
            gyroTurn(0.4,90);
            encoderMecanumDrive(1.0,50,10,0,-1);
            encoderMecanumDrive(1.0,220,10,1,0);
            spit();
            encoderMecanumDrive(DRIVE_SPEED,50,2,-1,0);


        } else if (location.equals("Left")){


            encoderMecanumDrive(DRIVE_SPEED,5,1,0,1);
            gyroTurn(TURN_SPEED,135);
            //encoderMecanumDrive(DRIVE_SPEED,8,2,0.93969,0.34202);
            succ();
            encoderMecanumDrive(0.2,60,10,1,1);
            succstop();
            gyroTurn(0.4,90);
            encoderMecanumDrive(1.0,50,10,0,-1);
            encoderMecanumDrive(1.0,220,10,1,0);
            spit();
            encoderMecanumDrive(DRIVE_SPEED,50,2,-1,0);


        }else if (location.equals("Center")){

            gyroTurn(TURN_SPEED,90);
            succ();
            encoderMecanumDrive(0.2,60,10,0,1);
            succstop();
            encoderMecanumDrive(1.0,50,10,0,-1);
            encoderMecanumDrive(1.0,220,10,1,0);
            spit();
            encoderMecanumDrive(DRIVE_SPEED,50,2,-1,0);
        }*/





    }
}