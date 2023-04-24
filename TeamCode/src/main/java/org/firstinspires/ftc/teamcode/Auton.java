package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

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

        robot.imu.resetYaw();



        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        ((DcMotorEx) robot.frontLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        ((DcMotorEx) robot.frontRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        ((DcMotorEx) robot.backLeft).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));
        ((DcMotorEx) robot.backRight).setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(20,0, 0, 10));

        DcMotor FL = robot.frontLeft, FR = robot.frontRight, BL = robot.backLeft, BR = robot.backRight;
        int FLpos = 0, FRpos = 0, BLpos = 0, BRpos = 0;

        waitForStart();
        while (opModeIsActive()){
            double Move1=2.4; // Need TUNE
            X=1000;

            robot.walkForward((int)(Move1 * X), 0.5);
            // walk to centre
            while (FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()){}

            double Move2 = 0.25;
            robot.rotateRight((int)(0.25*X), 0.5);
            while (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -40){}

            double Move3 = 0.25;
            robot.walkForward((int)(Move3*X), 0.5);

            robot.IntakeAndReady();
            sleep(1000);
            robot.raiseLift(2300);
            sleep(1000);
            robot.release();
            while(robot.vert.isBusy()){}

            robot.setMode("reset encoder");
            double Move4 = 2;
            robot.rotateLeft((int)(Move4*X), 0.5);

            while (robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < 90){}
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

        }
    }
}
