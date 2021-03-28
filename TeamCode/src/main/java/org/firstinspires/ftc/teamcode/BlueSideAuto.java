package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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

        VoltageSensor voltageSensor;
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        drivetrain = new Drivetrain(robot);
        sleep(250);

        //Set Angle based on voltage
        if (voltageSensor.getVoltage()<13.5){
            drivetrain.tilt(0.44);
        } else {
            drivetrain.tilt(0.45);
        }

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

        drivetrain.outtakeAll(1.0);//rev up
        if (voltageSensor.getVoltage()<13.5){
            encoderMecanumDrive(0.7, 215, 6, -0.464/*0.464*/,-1);
        } else {
            encoderMecanumDrive(0.8, 215, 6, -0.464/*0.464*/,-1);
        }
        gyroTurn(0.5,startingAngle);
//        encoderMecanumDrive(0.3, 10, 3, 1, 0);

        encoderMecanumDrive(0.4, 10, 3, -1, 0);//slide to reach 3rd power shot
        //Shoot 3
        drivetrain.invertSingleCycle();

        encoderMecanumDrive(0.4, 30, 3, 1, 0);//shoots 3rd then moves
        //Shoot 2
        drivetrain.invertSingleCycle();

        encoderMecanumDrive(0.4, 30, 3, 1, 0);//shoots 2nd then moves hopefully
        //Shoot 1
        drivetrain.invertSingleCycle();

        sleep(200);
//        drivetrain.outtakeAll(0.0);

        switch (numOfRings){
            case "FOUR":
                encoderMecanumDrive(0.8, 280, 3, 0.83, -1);
                inAndOutWobble();

                //END OF FIRST WOBBLE

                drivetrain.tilt(0.37);
                robot.intake.setPower(1);
                robot.wobble.setPosition(1);
                //come towards ring
                gyroTurn(0.7,startingAngle);
                encoderMecanumDrive(0.70, 164, 3, -0.2, 1);
                sleep(750);
                gyroTurn(0.7, startingAngle-10);
                encoderMecanumDrive(0.35,97.5,3,0.35,1);
//                gyroTurn(0.5,getAverageGyro()+15);
//                encoderMecanumDrive(0.45,30,3,-0.39,1);
                encoderMecanumDrive(0.50,8,3,0.2,-1);
//                gyroTurn(0.7,startingAngle-15);
//                encoderMecanumDrive(0.4,29.5,3,1,0);
//                //move out + back in to pick up wobble
//                sleep(400);
//                robot.wobble.setPosition(0);
//                sleep(150);
                if (voltageSensor.getVoltage()<13.5){
                    drivetrain.tilt(0.44);
                } else {
                    drivetrain.tilt(0.45);
                }
                gyroTurn(0.5, startingAngle+5);
                encoderMecanumDrive(0.8, 40, 3, 0, -1);
                gyroTurn(0.5, startingAngle+5);
                robot.intake.setPower(0);
                drivetrain.invertSingleCycle();
                sleep(50);
                drivetrain.invertSingleCycle();
                sleep(50);
                drivetrain.invertSingleCycle();
                encoderMecanumDrive(0.4, 20, 3, 0, -1);


//                drivetrain.waitTillPos(robot.wobble,0);
//                encoderMecanumDrive(0.8, 270, 3, 0.2, -1);
//                inAndOutWobble();
//                encoderMecanumDrive(0.8, 100, 3, 0, 1);





                break;
            case "ONE":
                encoderMecanumDrive(0.8, 145, 3, -0.3, -1);//forward
                inAndOutWobble();

//                encoderMecanumDrive(0.8, 50, 3, 0, 1);//move to line

                //END OF FIRST WOBBLE
                drivetrain.tilt(0.37);
                robot.intake.setPower(1);
                robot.wobble.setPosition(1);
                //come towards ring
                encoderMecanumDrive(0.6,227.5,3,0.575,1);
                sleep(750);
                robot.intake.setPower(0);
                encoderMecanumDrive(0.8,87.5,3,-0.39,1);
                encoderMecanumDrive(0.3,29.5,3,1,0);
                //move out + back in to pick up wobble
                sleep(500);
                robot.wobble.setPosition(0);
                sleep(150);
                drivetrain.waitTillPos(robot.wobble,0);
                //drive back to zone, turn, and drop (also park)
                encoderMecanumDrive(0.72,168,3,-0.28,-1);
                gyroTurn(0.7,getAverageGyro()-90);
                inAndOutWobble();
                robot.outtake.setPower(1.0);
                if (voltageSensor.getVoltage()<13.5){
                    drivetrain.tilt(0.43);
                } else {
                    drivetrain.tilt(0.44);
                }
                gyroTurn(0.5, startingAngle+10);
                encoderMecanumDrive(0.5, 20, 3, 0, 1);
                drivetrain.invertSingleCycle();
                sleep(50);
                drivetrain.invertSingleCycle();
                sleep(50);
                drivetrain.invertSingleCycle();
                sleep(50);
                encoderMecanumDrive(0.5, 20, 3, 0, -1);
//                encoderMecanumDrive(0.5,15,3,0,1);
                robot.outtake.setPower(0.0);
                break;
            case "NONE":
                encoderMecanumDrive(0.5, 105, 3, 1, -0.6);
                inAndOutWobble();
                encoderMecanumDrive(0.5, 40, 3, -1, 0);
                gyroTurn(0.6,startingAngle);
                robot.wobble.setPosition(1);

                //END OF FIRST WOBBLE

                encoderMecanumDrive(0.75, 150, 3, 0, 1);
                encoderMecanumDrive(0.3,25,3,1,0);
                //move out + back in to pick up wobble
                sleep(500);
                robot.wobble.setPosition(0);
                sleep(150);
                drivetrain.waitTillPos(robot.wobble,0);
                encoderMecanumDrive(0.75, 160, 3, 0.2, -1);
                inAndOutWobble();

                break;
        }
//        robot.wobble.setPosition(0.0);
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
        drivetrain.outtakeAll(0.0);
        encoderMecanumDrive(0.4, 20, 3, 1, 0);
        drivetrain.waitTillPos(robot.wobble, 1.0);
        encoderMecanumDrive(0.4, 30, 3, -1, 0);
        sleep(100);
        gyroTurn(0.5 , getAverageGyro()-30);
        sleep(100);
        robot.wobble.setPosition(0.0);
        gyroTurn(0.5, startingAngle);
        sleep(400);

    }
}