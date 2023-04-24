package org.firstinspires.ftc.teamcode;

public class snippets {

    /*

            PIDFCoefficients pidfNew = new PIDFCoefficients(3,0,0,0);
            PIDFCoefficients pidf = robot.arm.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);

            telemetry.addData("original:", "%.04f, %.04f, %.04f, %.04f", pidf.p, pidf.i, pidf.d, pidf.f);
     */

            /*
            double Kp, Ki, Kd;
            double error = 0, lastError = 0, integralSum = 0;
            double out = 0;
            int cur = 0, reference = 30;
            ElapsedTime timer = new ElapsedTime();

            Kp = 0.005; Ki = 0; Kd = 0;


            if (gamepad1.right_bumper){
                timer.reset();
                while (1 == 1){
                    if (gamepad1.triangle){
                        reference += 10;
                        sleep(500);
                    }
                    cur = arm.getCurrentPosition();
                    telemetry.addData("Encoder value", arm.getCurrentPosition());
                    telemetry.update();
                    lastError = error;
                    error = reference-cur;
                    integralSum += error * timer.seconds();

                    //out = (error * Kp) + (integralSum * Ki) + ( ((error - lastError) / timer.seconds())  * Kd);

                    arm.setPower((error * Kp));

                    timer.reset();
                }

            }
            */ // custom PID

            /*
            while (gamepad1.dpad_up || gamepad1.dpad_down){
                if (gamepad1.dpad_up) pidfNew.p = pidfNew.p + 0.1;
                else if (gamepad1.dpad_down) pidfNew.p = pidfNew.p - 0.1;
            }

            if (gamepad1.right_bumper){
                robot.arm.setTargetPosition((int)(TICKS_PER_DEGREE * 100));
                robot.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.arm.setPower(0.5);
            }
            */ // rev PID with setVelocity() and custom PIDF coefficients

}



/*
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

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


        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        //((DcMotorEx) robot.frontLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        //((DcMotorEx) robot.frontRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        //((DcMotorEx) robot.backLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        //((DcMotorEx) robot.backRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));



        DcMotor FL = robot.frontLeft, FR = robot.frontRight, BL = robot.backLeft, BR = robot.backRight;
        int FLpos = 0, FRpos = 0, BLpos = 0, BRpos = 0;


        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.claw1.setPosition(0.95);
        robot.claw2.setPosition(0.75);
        robot.yaw2.setPosition(-0.4);
        robot.vert.setTargetPosition(450);

        waitForStart();
        while (opModeIsActive()){
            robot.vert.setTargetPosition(850);
            robot.vert.setPower(0.8);
            X=1000;
            double Move1=3; // Need TUNE
            FLpos += (int)(Move1*X);
            FRpos -= (int)(Move1*X);
            BLpos -= (int)(Move1*X);
            BRpos += (int)(Move1*X);

            robot.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.yaw1.setPosition(0.65);
            robot.yaw2.setPosition(0.05);

            while (FL.getCurrentPosition()<2999){
                telemetry.addData("LF", FL.getCurrentPosition());
                telemetry.update();
            }
            robot.setMotorPowers(0,0,0,0);


        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            double Move2=0.35; // Need TUNE
            FLpos = (int)(Move2*X);
            FRpos = (int)(Move2*X);
            BLpos = (int)(Move2*X);
            BRpos = (int)(Move2*X);

            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.setMotorPowers(0.3,0.3,0.3,0.3);

            sleep(2000);

            robot.claw1.setPosition(0.8);
            robot.claw2.setPosition(1);
            robot.yaw1.setPosition(0.4);
            robot.yaw2.setPosition(0.3);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            double Move3=0.6; // Need TUNE
            FLpos = -(int)(Move3*X);
            FRpos = -(int)(Move3*X);
            BLpos = -(int)(Move3*X);
            BRpos = -(int)(Move3*X);

            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.setMotorPowers(0.3,0.3,0.3,0.3);

            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            double Move4=2; // Need TUNE
            FLpos = (int)(Move4*X);
            FRpos = -(int)(Move4*X);
            BLpos = -(int)(Move4*X);
            BRpos = (int)(Move4*X);

            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.setMotorPowers(0.3,0.3,0.3,0.3);
            sleep(10000000);
            /*

            robot.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(500);
            robot.vert.setTargetPosition(500);
            robot.vert.setPower(0.8);
            robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.yaw1.setPosition(0.4);
            robot.yaw2.setPosition(0.3);

            robot.yaw1.setPosition(0.65);
            robot.yaw2.setPosition(0.05);
            robot.claw1.setPosition(0.95);
            robot.claw2.setPosition(0.75);

            sleep(2000);

            double Move3=3; // Need TUNE
            FLpos += (int)(Move3*X);
            FRpos -= (int)(Move3*X);
            BLpos -= (int)(Move3*X);
            BRpos += (int)(Move3*X);

            robot.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);



            sleep(50000);

            /*
            double Move1=2.45; // Need TUNE
            FLpos += (int)(Move1*X);
            FRpos += (int)(Move1*X);
            BLpos += (int)(Move1*X);
            BRpos += (int)(Move1*X);

            robot.setMotorPowers(0.3, 0.3, 0.3, 0.3);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            // walk to centre

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}
            sleep(500);

            FLpos = robot.frontLeft.getCurrentPosition();
            FRpos = robot.frontRight.getCurrentPosition();
            BLpos = robot.backLeft.getCurrentPosition();
            BRpos = robot.backRight.getCurrentPosition();


            double Move2 = 0.47;
            FLpos -= (int)(Move2 * X);
            FRpos += (int)(Move2 * X);
            BLpos -= (int)(Move2 * X);
            BRpos += (int)(Move2 * X);
            robot.setMotorPowers(0.3, 0.25, 0.5, 0.45);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -40){
                //telemetry.addData("gyro", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                //telemetry.update();
                robot.vert.setPower(0);
            }
            robot.setMotorPowers(0, 0, 0, 0);


            robot.vert.setTargetPosition(2400);
            robot.vert.setPower(0.8);
            robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.yaw2.setPosition(-0.4);
            sleep(2000);

            robot.claw1.setPosition(0.8);
            robot.claw2.setPosition(1);
            sleep(1000);

            robot.vert.setTargetPosition(150);

            double Move3 = 0.47;
            FLpos += (int)(Move2 * X);
            FRpos -= (int)(Move2 * X);
            BLpos += (int)(Move2 * X);
            BRpos -= (int)(Move2 * X);
            robot.setMotorPowers(0.3, 0.3, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 0){
                //telemetry.addData("gyro", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                //telemetry.update();
                robot.vert.setPower(0);
            }
            robot.setMotorPowers(0, 0, 0, 0);

            double Move4=1.25; // Need TUNE
            FLpos -= (int)(Move4*X);
            FRpos -= (int)(Move4*X);
            BLpos -= (int)(Move4*X);
            BRpos -= (int)(Move4*X);

            robot.setMotorPowers(0.3, 0.3, 0.4, 0.4);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            // walk to centre

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(2000);
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}
            sleep(50000);


            /*

            FLpos = FL.getCurrentPosition();
            FRpos = FR.getCurrentPosition();
            BLpos = BL.getCurrentPosition();
            BRpos = BR.getCurrentPosition();

            double Move3=0.05; // Need TUNE
            FLpos += (int)(Move1*X);
            FRpos += (int)(Move1*X);
            BLpos += (int)(Move1*X);
            BRpos += (int)(Move1*X);

            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            // walk to centre

            FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}
            robot.setMotorPowers(0,0,0,0);

            robot.yaw2.setPosition(0);
            sleep(200);
            robot.claw1.setPosition(0.8);
            robot.claw2.setPosition(1);

            robot.vert.setTargetPosition(150);

        /*
         double Move7 = 0.47;
            FLpos += (int)(Move2 * X);
            FRpos -= (int)(Move2 * X);
            BLpos += (int)(Move2 * X);
            BRpos -= (int)(Move2 * X);
            robot.setMotorPowers(0.5, 0.5, 0.3, 0.3);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > 130){
                //telemetry.addData("gyro", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                //telemetry.update();
                robot.vert.setPower(0);

            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            // return to refill station

            robot.claw1.setPosition(0.95);
            robot.claw2.setPosition(0.75);

            robot.yaw1.setPosition(0.35);
            robot.yaw2.setPosition(0.25);

            double Move8 = 0.47;
            FLpos -= (int)(Move2 * X);
            FRpos += (int)(Move2 * X);
            BLpos -= (int)(Move2 * X);
            BRpos += (int)(Move2 * X);
            robot.setMotorPowers(0.3, 0.3, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -130){
                //telemetry.addData("gyro", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                //telemetry.update();
                robot.vert.setPower(0);

            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            robot.setMotorPowers(0, 0, 0, 0);

            robot.vert.setTargetPosition(2300);
            robot.vert.setPower(0.8);
            robot.vert.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.yaw2.setPosition(-0.4);
            while(robot.vert.isBusy()){}

            robot.yaw2.setPosition(0);
            sleep(200);
            robot.claw1.setPosition(0.95);
            robot.claw2.setPosition(0.75);

            robot.vert.setTargetPosition(150);

            //ready for parking

            double Move11 = 0.47;
            FLpos -= (int)(Move2 * X);
            FRpos += (int)(Move2 * X);
            BLpos -= (int)(Move2 * X);
            BRpos += (int)(Move2 * X);
            robot.setMotorPowers(0.3, 0.3, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -145){
                //telemetry.addData("gyro", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                //telemetry.update();
                robot.vert.setPower(0);

            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            double Move12 = 0.47;
            FLpos -= (int)(Move2 * X);
            FRpos += (int)(Move2 * X);
            BLpos -= (int)(Move2 * X);
            BRpos += (int)(Move2 * X);
            robot.setMotorPowers(0.3, 0.3, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            while (robot.imu1.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -90){
                //telemetry.addData("gyro", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
                //telemetry.update();
                robot.vert.setPower(0);

            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            sleep(10000);

            //while(robot.vert.isBusy()){}

            /*
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}
            FLpos = robot.frontLeft.getCurrentPosition();
            FRpos = robot.frontRight.getCurrentPosition();
            BLpos = robot.backLeft.getCurrentPosition();
            BRpos = robot.backRight.getCurrentPosition();

            double Move3 = 0.05;
            FLpos += (int)(Move3 * X);
            FRpos += (int)(Move3 * X);
            BLpos += (int)(Move3 * X);
            BRpos += (int)(Move3 * X);
            robot.setMotorPowers(0.5, 0.5, 0.5, 0.5);
            sleep(700);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);

            //sleep(2000);

            // rotate right by a bit to align with junction
            /*


            double Move3 = 0.75;
            FLpos += (int)(Move3 * X);
            FRpos -= (int)(Move3 * X);
            BLpos -= (int)(Move3 * X);
            BRpos += (int)(Move3 * X);


            robot.setMotorPowers(-0.5, 0.5, 0.5, -0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            //rotate ~120 degrees to face the refill substation

            double Move4 = 2.25;
            FLpos += (int)(Move4 * X);
            FRpos += (int)(Move4 * X);
            BLpos += (int)(Move4 * X);
            BRpos += (int)(Move4 * X);

            robot.setMotorPowers(0.8, 0.8, 0.8, 0.8);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            // go to substation

            robot.claw1.setPosition(0.8);
            robot.claw2.setPosition(1);
            robot.yaw2.setPosition(0.24);
            robot.vert.setTargetPosition(150);
            sleep(200);
            // pickup cone

            double Move5 = 1;
            FLpos += (int)(Move5 * X);
            FRpos -= (int)(Move5 * X);
            BLpos -= (int)(Move5 * X);
            BRpos += (int)(Move5 * X);

            robot.setMotorPowers(0.5, -0.5, -0.5, 0.5);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            // rotate by 180 degrees

            double Move6 = 2.25;
            FLpos += (int)(Move6 * X);
            FRpos += (int)(Move6 * X);
            BLpos += (int)(Move6 * X);
            BRpos += (int)(Move6 * X);

            robot.setMotorPowers(0.8, 0.8, 0.8, 0.8);
            robot.setMotorPositions(FLpos, FRpos, BLpos, BRpos);
            // leave substation

        }
                }

                }
 */