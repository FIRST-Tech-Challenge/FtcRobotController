package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "BlueSideAuto",group = "SkyStone")
public class BlueSideAuto extends BaseAuto {

    Drivetrain drivetrain;
    String numOfRings = "";
    double startingAngle;


    @Override
    public void runOpMode() throws InterruptedException {

        //Initialize the hardware using BaseAutonomous Function
        inithardware();

        initializeOpenCV();

        drivetrain = new Drivetrain(robot);
        sleep(250);
        drivetrain.tilt(0.44);
        sleep(250);
//        drivetrain.moveSlapper(Drivetrain.slapperPos.IN);

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

        sleep(500);
        telemetry.addData("Position", pipeline.position);
        telemetry.update();

        sleep(500);
        if (pipeline.position.toString() == "FOUR"){
            telemetry.addData("Going with four", "");
        } else if (pipeline.position.toString() == "ONE"){
            telemetry.addData("Going with one", "");
        } else {
            telemetry.addData("Going with none", "");
        }
        telemetry.update();
        numOfRings = pipeline.position.toString();

        startingAngle = getAverageGyro();
        encoderMecanumDrive(0.8, 215, 6, -0.464,-1);
//        encoderMecanumDrive(0.3, 10, 3, 1, 0);
        drivetrain.outtakeAll(1.0);//rev up

        encoderMecanumDrive(0.4, 10, 3, -1, 0);//slide to reach 3rd power shot
        //Shoot 3
        drivetrain.invertSingleCycle();

        encoderMecanumDrive(0.4, 30, 3, 1, 0);//shoots 3rd then moves
        //Shoot 2
        drivetrain.invertSingleCycle();

        encoderMecanumDrive(0.4, 30, 3, 1, 0);//shoots 2nd then moves hopefully
        //Shoot 1
        drivetrain.invertSingleCycle();

        drivetrain.outtakeAll(0.0);

        switch (numOfRings){
            case "FOUR":
                encoderMecanumDrive(0.8, 190, 3, 0.5, -1);
                inAndOutWobble();
                encoderMecanumDrive(0.8, 100, 3, 0, 1);

                //END OF FIRST WOBBLE


                break;
            case "ONE":
                encoderMecanumDrive(0.8, 130, 3, -0.3, -1);//forward
                inAndOutWobble();
                encoderMecanumDrive(0.8, 50, 3, 0, 1);//move to line

                //END OF FIRST WOBBLE


                break;
            case "NONE":
                encoderMecanumDrive(0.5, 50, 3, 1, -0.6);
                inAndOutWobble();
                encoderMecanumDrive(0.5, 20, 3, -1, 0);

                //END OF FIRST WOBBLE
                break;
        }
        robot.wobble.setPosition(0.0);
        gyroTurn(0.5, startingAngle);
        sleep(300);

//        sleep(300);
//        encoderMecanumDrive(0.8, 120, 5, 0, 1);
//        sleep(500);
//        encoderMecanumDrive(0.5, 90, 3, 1, 0);


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

    public void inAndOutWobble(){
        encoderMecanumDrive(0.4, 20, 3, 1, 0);
        sleep(100);
        robot.wobble.setPosition(1.0);
        sleep(600);
        encoderMecanumDrive(0.4, 30, 3, -1, 0);
        sleep(100);
        gyroTurn(0.3, getAverageGyro()-30);
        sleep(100);
        gyroTurn(0.3, startingAngle);
    }
}