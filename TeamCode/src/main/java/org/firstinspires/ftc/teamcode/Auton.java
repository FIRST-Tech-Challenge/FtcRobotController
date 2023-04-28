package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name="jerry")
public class Auton extends LinearOpMode {

    final static double ARM_HOME = 0.0;
    final static double ARM_MIN_RANGE = 0.0;
    final static double ARM_MAX_RANGE = 1.0;
    final static double HEXMOTOR_TPR = 288;
    final static double GOBILDA_TPR = 751.8; //5202
    final static double GEAR_REDUCTION = 1;
    final static double TICKS_PER_DEGREE = (HEXMOTOR_TPR * GEAR_REDUCTION) / 360;
    double X=0; //square distance for run to pos
    //ticks = pulses, cycles = ticks * 4

    @Override
    public void runOpMode() throws InterruptedException {
        Project1Hardware robot = new Project1Hardware();
        MecanumDrive drivetrain  = new MecanumDrive(robot);

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        robot.imu1.resetYaw();

        robot.vert.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        ((DcMotorEx) robot.frontLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        ((DcMotorEx) robot.frontRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        ((DcMotorEx) robot.backLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        ((DcMotorEx) robot.backRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));

        DcMotor FL = robot.frontLeft, FR = robot.frontRight, BL = robot.backLeft, BR = robot.backRight;
        int FLpos = 0, FRpos = 0, BLpos = 0, BRpos = 0;
        robot.vert.setPower(0);
        robot.raiseLift(0);
        robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.IntakeClose();
        robot.yaw1.setPosition(0.65);
        robot.yaw2.setPosition(0.05);
        robot.setMode(1);
        waitForStart();
        while (opModeIsActive()){
            double gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*1;
            double lastpos = FL.getCurrentPosition();
            robot.imu1.resetYaw();

            sleep(500);
            robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.raiseLift(600);

            double Move1=1.7; // Need TUNE
            X=1300;



            robot.moveRight((int)(Move1 * X), 0.5, 0.5, 0.5, 0.5);
            robot.IntakeClose();
            // walk to centre
            while (BR.getCurrentPosition() < (Move1 * X-200)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0);
            sleep(200);




            robot.setMode(1);
            robot.setMode(3);

            while (BL.getCurrentPosition() < 150){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;

                robot.setMotorPowers((0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("BL1",BL.getCurrentPosition());

                telemetry.update();
            }

            robot.setMotorPowers(0);
            /*
            sleep(1000);
            robot.yaw1.setPosition(0.4);
            robot.yaw2.setPosition(0.3);
            sleep(500);
            robot.claw1.setPosition(0.8);
            robot.claw2.setPosition(1);
            sleep(500);
            */
            robot.IntakeOpen();
            sleep(500);
            robot.raiseLift(1200);
            robot.setMode(2);

            robot.moveRight(850, 0.5, 0.5, 0.5, 0.5);

            lastpos = FL.getCurrentPosition();

            // walk to centre
            while (FL.getCurrentPosition() < (lastpos + 650)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0);
            telemetry.addData("imu", robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            sleep(200);
            /*
            if (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < -1){
                robot.rotateLeft(300, 0.1);
                while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 0){
                    telemetry.addData("imu", robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                }
            }
            else{
                robot.rotateRight(300, 0.1);
                while (Math.abs(robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)) > 1){
                    telemetry.addData("imu", robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                    telemetry.update();
                }
            }*/
            //double gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)/ 360;


            robot.setMode(1);
            robot.setMode(3);
            robot.raiseLift(430);


            while (BL.getCurrentPosition() < 1100){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;

                robot.setMotorPowers((0.5-gyrocal),(0.5+gyrocal),(0.5-gyrocal),(0.5+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
            }

            robot.setMotorPowers(0);



            robot.IntakeClose();
            sleep(500);
            robot.raiseLift(1000);
            sleep(800);
            robot.setMode(2);
            robot.setMode(1);
            robot.walkBackward(2500, 0.5);



            while (FL.getCurrentPosition() > -2300){
                //if (FL.getCurrentPosition() < (lastpos-1600)) robot.setMotorPowers(0.4);
                //else if (FL.getCurrentPosition() < (lastpos-2000)) robot.setMotorPowers(0.3);
                telemetry.addData("FL2", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.yaw1.setPosition(0.65);
            robot.yaw2.setPosition(0.05);
            robot.setMotorPowers(0);
            robot.raiseLift(4000);
            robot.setMode(1);
            robot.setMode(3);


            while (BL.getCurrentPosition() > (-800)){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
                telemetry.addData("BL1",BL.getCurrentPosition());
            }
            robot.setMotorPowers(0);
            robot.setMode(1);
            robot.setMode(3);
            while (BL.getCurrentPosition() < 220){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("BLfw",BL.getCurrentPosition());

                telemetry.update();
            }

            robot.setMotorPowers(0);
            while (robot.vert.getCurrentPosition() < 2000){}
            sleep(300);
            robot.IntakeOpen();
            sleep(300);

            robot.setMode(1);
            robot.setMode(3);
            while (BL.getCurrentPosition() > -220){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((-0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(-0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("BLfw",BL.getCurrentPosition());

                telemetry.update();
            }
            robot.setMode(1);
            while (BL.getCurrentPosition() < 700){
                robot.setMode(3);
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((-0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(-0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
                telemetry.addData("BL1",BL.getCurrentPosition());
            }

            robot.setMotorPowers(0);
            robot.setMode(1);
            robot.setMode(3);
            robot.raiseLift(300);
            while (BL.getCurrentPosition() < 2600){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((0.5-gyrocal),(0.5+gyrocal),(0.5-gyrocal),(0.5+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("BLfw",BL.getCurrentPosition());

                telemetry.update();
            }
            robot.setMotorPowers(0);

            //second cycle

            robot.IntakeClose();
            sleep(500);
            robot.raiseLift(1000);
            sleep(800);
            robot.setMode(2);
            robot.setMode(1);
            robot.walkBackward(2500, 0.5);



            while (FL.getCurrentPosition() > -2300){
                //if (FL.getCurrentPosition() < (lastpos-1600)) robot.setMotorPowers(0.4);
                //else if (FL.getCurrentPosition() < (lastpos-2000)) robot.setMotorPowers(0.3);
                telemetry.addData("FL2", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.yaw1.setPosition(0.65);
            robot.yaw2.setPosition(0.05);
            robot.setMotorPowers(0);
            robot.raiseLift(4000);
            robot.setMode(1);
            robot.setMode(3);


            while (BL.getCurrentPosition() > (-800)){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
                telemetry.addData("BL1",BL.getCurrentPosition());
            }
            robot.setMotorPowers(0);
            robot.setMode(1);
            robot.setMode(3);
            while (BL.getCurrentPosition() < 220){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("BLfw",BL.getCurrentPosition());

                telemetry.update();
            }

            robot.setMotorPowers(0);
            while (robot.vert.getCurrentPosition() < 2000){}
            sleep(300);
            robot.IntakeOpen();
            sleep(300);

            robot.setMode(1);
            robot.setMode(3);
            while (BL.getCurrentPosition() > -220){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((-0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(-0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("BLfw",BL.getCurrentPosition());

                telemetry.update();
            }
            robot.setMode(1);
            while (BL.getCurrentPosition() < 700){
                robot.setMode(3);
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((-0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(-0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
                telemetry.addData("BL1",BL.getCurrentPosition());
            }

            robot.setMotorPowers(0);
            robot.setMode(1);
            robot.setMode(3);
            robot.raiseLift(300);
            while (BL.getCurrentPosition() < 2600){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((0.5-gyrocal),(0.5+gyrocal),(0.5-gyrocal),(0.5+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("BLfw",BL.getCurrentPosition());

                telemetry.update();
            }
            robot.setMotorPowers(0);

            robot.walkBackward(2500, 0.5);

            while (FL.getCurrentPosition() > -2300){
                //if (FL.getCurrentPosition() < (lastpos-1600)) robot.setMotorPowers(0.4);
                //else if (FL.getCurrentPosition() < (lastpos-2000)) robot.setMotorPowers(0.3);
                telemetry.addData("FL2", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.yaw1.setPosition(0.65);
            robot.yaw2.setPosition(0.05);
            robot.setMotorPowers(0);
            robot.raiseLift(4000);
            robot.setMode(1);
            robot.setMode(3);


            while (BL.getCurrentPosition() > (-800)){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.update();
                telemetry.addData("BL1",BL.getCurrentPosition());
            }
            robot.setMotorPowers(0);
            robot.setMode(1);
            robot.setMode(3);
            while (BL.getCurrentPosition() < 220){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((0.3-gyrocal),(0.3+gyrocal),(0.3-gyrocal),(0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("BLfw",BL.getCurrentPosition());

                telemetry.update();
            }

            robot.setMotorPowers(0);
            while (robot.vert.getCurrentPosition() < 2000){}
            sleep(300);
            robot.IntakeOpen();
            sleep(300);

            robot.setMode(1);
            robot.setMode(3);
            while (BL.getCurrentPosition() > -220){
                gyrocal = robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES)*4/90;
                robot.setMotorPowers((-0.3-gyrocal),(-0.3+gyrocal),(-0.3-gyrocal),(-0.3+gyrocal));
                telemetry.addData("gyro",robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                telemetry.addData("BLfw",BL.getCurrentPosition());

                telemetry.update();
            }
            robot.setMode(1);

            /*
            sleep(1000);

            // walk to centre
            robot.setMotorPowers(0);
            robot.raiseLift(400);
            robot.setMode(2);
            robot.walkForward(1200, 0.3);

            //robot.IntakeClose();
            lastpos = FL.getCurrentPosition();

            while (FL.getCurrentPosition() < (lastpos + 950)){
                if (lastpos - FL.getCurrentPosition() < 400) robot.setMotorPowers(0.3);
            }

            robot.setMotorPowers(0);
            sleep(500);
            robot.IntakeClose();
            sleep(500);
            robot.yaw1.setPosition(0.65);
            robot.yaw2.setPosition(0.05);

            robot.setMotorPowers(0);
            robot.setMode(2);
            sleep(1000);
            robot.walkBackward(2500, 0.5);

            robot.raiseLift(2400);

            lastpos = FL.getCurrentPosition();

            while (FL.getCurrentPosition() > (lastpos - 2300)){
                if (FL.getCurrentPosition() < (lastpos-1600)) robot.setMotorPowers(0.4);
                else if (FL.getCurrentPosition() < (lastpos-2000)) robot.setMotorPowers(0.3);
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }

            robot.setMotorPowers(0);
            robot.setMode(1);
            sleep(1000);

            robot.moveRight(1000, 0.3, 0.3, 0.3, 0.3);
            robot.IntakeClose();
            robot.yaw1.setPosition(0.7);
            robot.yaw2.setPosition(0.0);
            // walk to centre
            lastpos = BR.getCurrentPosition();

            while (BR.getCurrentPosition() < (lastpos+600)){}
            robot.setMotorPowers(0);
            sleep(500);

            robot.walkForward(250, 0.3);

            //robot.IntakeClose();

            lastpos = FL.getCurrentPosition();

            while (FL.getCurrentPosition() < (lastpos + 220)){}

            robot.setMotorPowers(0);
            robot.yaw1.setPosition(0.4);
            robot.yaw2.setPosition(0.3);
            robot.IntakeOpen();
            robot.raiseLift(300);

            sleep(5000);

            robot.walkBackward(180, 0.3);

            lastpos = FL.getCurrentPosition();

            while (FL.getCurrentPosition() < (lastpos + 180 - 30)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0);
            sleep(800);

            robot.moveLeft(600, 0.5, 0.5, 0.5, 0.5);
            // walk to centre
            lastpos = FL.getCurrentPosition();

            while (BR.getCurrentPosition() < (lastpos-20)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0);
            sleep(800);

            /*
            robot.walkBackward(180, 0.3);

            lastpos = FL.getCurrentPosition();

            while (FL.getCurrentPosition() < (lastpos + 180 - 30)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0);
            sleep(800);

            robot.moveLeft(600, 0.5, 0.5, 0.5, 0.5);
            // walk to centre
            lastpos = FL.getCurrentPosition();

            while (BR.getCurrentPosition() < (lastpos-20)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0);
            sleep(800);

            robot.moveForward(2680, 0.5, 0.5, 0.5, 0.5);
            // walk to centre
            lastpos = FL.getCurrentPosition();

            while (BR.getCurrentPosition() < (lastpos-20)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0);
            sleep(1600);

            robot.IntakeClose();
            sleep(500);
            robot.yaw1.setPosition(0.65);
            robot.yaw2.setPosition(0.05);
            robot.raiseLift(2400);

            robot.setMotorPowers(0);
            robot.setMode(2);
            sleep(1000);
            robot.walkBackward(2700, 0.5);

            lastpos = FL.getCurrentPosition();

            while (FL.getCurrentPosition() > (lastpos - 2400)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0);
            robot.setMode(1);
            sleep(1000);

            robot.moveRight(700, 0.3, 0.3, 0.3, 0.3);
            robot.IntakeClose();
            // walk to centre
            lastpos = BR.getCurrentPosition();

            while (BR.getCurrentPosition() < lastpos-100){}
            robot.setMotorPowers(0);
            sleep(500);

            robot.walkForward(180, 0.3);

            //robot.IntakeClose();

            lastpos = FL.getCurrentPosition();

            while (FL.getCurrentPosition() < (lastpos + 180 - 30)){}

            robot.setMotorPowers(0);
            robot.IntakeOpen();
            robot.raiseLift(450);


            double Move2 = 0.45;
            robot.moveLeft((int)(Move2 * X), 0.5, 0.5, 0.5, 0.5);
            robot.setMode(2);
            //robot.IntakeClose();
            /*
            while (FL.getCurrentPosition() < (Move2 * X-200)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.moveLeft(2700, 0.5);

            lastpos = FL.getCurrentPosition();

            while (FL.getCurrentPosition() > (lastpos - 2400)){
                telemetry.addData("FL", FL.getCurrentPosition());
                telemetry.addData("FR", FR.getCurrentPosition());
                telemetry.addData("BL", BL.getCurrentPosition());
                telemetry.addData("BR", BR.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0);
            robot.setMode(1);
            sleep(1000);


            sleep(100000000);

           /* double Move2 = 1;
            robot.rotateLeft((int)(1*X), 0.5);
            while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 15){}

            robot.setMotorPowers(0);
            sleep(1000);

            double Move3 = 0.1;
            robot.walkForward((int)(Move3*X), 0.3);

            /*
            robot.IntakeClose();
            sleep(1000);
            robot.raiseLift(2300);
            sleep(1000);
            robot.release();
            while(robot.vert.isBusy()){}
            */

            /*
            double Move4 = 2;
            robot.rotateLeft((int)(Move4*X), 0.5);

            while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 180){}
            //rotate ~120 degrees to face the refill substation

            robot.setMotorPowers(0,0,0,0);
            sleep(200);

            double Move5 = 2.25;
            robot.walkForward((int)(Move5*X), 0.5);
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}

            robot.setMotorPowers(0,0,0,0);
            // go to substation

            sleep(50000);
            /*
            robot.claw1.setPosition(0.8);
            robot.claw2.setPosition(1);
            robot.yaw2.setPosition(0.24);
            robot.vert.setTargetPosition(150);
            sleep(200);
            // pickup cone

            robot.resetMotorEncoders();
            double Move6 = 2.25;
            FLpos -= (int)(Move6 * X);
            FRpos -= (int)(Move6 * X);
            BLpos -= (int)(Move6 * X);
            BRpos -= (int)(Move6 * X);
            robot.setMotorPowers(0.8, 0.8, 0.8, 0.8);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            //move away from substation, facing substation
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}
            robot.setMotorPowers(0,0,0,0);
            sleep(500);

            robot.resetMotorEncoders();
            double Move7 = 1;
            FLpos -= (int)(Move7 * X);
            FRpos += (int)(Move7 * X);
            BLpos += (int)(Move7 * X);
            BRpos -= (int)(Move7 * X);
            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            //move left
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}
            robot.setMotorPowers(0,0,0,0);
            sleep(500);


            robot.resetMotorEncoders();
            double Move8 = 0.25;
            FLpos += (int)(Move8 * X);
            FRpos += (int)(Move8 * X);
            BLpos += (int)(Move8 * X);
            BRpos += (int)(Move8 * X);
            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            //move left


            robot.vert.setTargetPosition(2000);
            robot.yaw2.setPosition(0);
            while (robot.vert.getCurrentPosition() < 2000){}

            robot.claw1.setPosition(0.95);
            robot.claw2.setPosition(0.75);
            //open
            robot.vert.setTargetPosition(0);

            robot.resetMotorEncoders();
            double Move9 = 1;
            FLpos += (int)(Move9 * X);
            FRpos -= (int)(Move9 * X);
            BLpos -= (int)(Move9 * X);
            BRpos += (int)(Move9 * X);
            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            //move left
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}
            robot.setMotorPowers(0,0,0,0);
            sleep(500);

            robot.resetMotorEncoders();
            double Move10 = 3;
            FLpos += (int)(Move10 * X);
            FRpos += (int)(Move10 * X);
            BLpos += (int)(Move10 * X);
            BRpos += (int)(Move10 * X);
            robot.setMotorPowers(0.8, 0.8, 0.8, 0.8);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            //move to substation

            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}
            robot.setMotorPowers(0,0,0,0);
            sleep(500);

            robot.resetMotorEncoders();
            double Move11 = 2.25;
            FLpos -= (int)(Move11 * X);
            FRpos -= (int)(Move11 * X);
            BLpos -= (int)(Move11 * X);
            BRpos -= (int)(Move11 * X);
            robot.setMotorPowers(0.8, 0.8, 0.8, 0.8);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            //move away from substation, facing substation
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}
            robot.setMotorPowers(0,0,0,0);
            sleep(500);

            robot.resetMotorEncoders();
            double Move12 = 1;
            FLpos -= (int)(Move12 * X);
            FRpos += (int)(Move12 * X);
            BLpos += (int)(Move12 * X);
            BRpos -= (int)(Move12 * X);
            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            //move left

            robot.vert.setTargetPosition(2000);
            robot.yaw2.setPosition(0);
            while (robot.vert.getCurrentPosition() < 2000){}

            robot.claw1.setPosition(0.95);
            robot.claw2.setPosition(0.75);
            //open
            robot.vert.setTargetPosition(0);

            robot.resetMotorEncoders();
            double Move13 = 1;
            FLpos += (int)(Move13 * X);
            FRpos -= (int)(Move13 * X);
            BLpos -= (int)(Move13 * X);
            BRpos += (int)(Move13 * X);
            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            //move left

             */
            sleep(1000000);

        }
    }
}
