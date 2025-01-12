/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//MINE ( AARUSH )
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.CommonUtil;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.

 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.

 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Unit_Test2", group="Linear Opmode2")
public class UnitTest2 extends LinearOpMode {

    double ENC2DIST = 2000.0/3.9558976; //2000.0/48.0; // FW/BW
    double ENC2DIST_SIDEWAYS = 2000.0/3.9558976;
    ElapsedTime timer = new ElapsedTime();

    //imu init
    BHI260IMU imu;
    BHI260IMU.Parameters myIMUParameters;
    YawPitchRollAngles robotOrientation;

    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;

    Orientation myRobotOrientation;

    @Override
    public void runOpMode() {


        // initialize hardware
        telemetry.setAutoClear(true);

        initialize(hardwareMap);
        // Initialize motors

        //resetMotorEncoderCounts();
        setMotorToZeroPower();

        Orientation myRobotOrientation;


        bl = hardwareMap.get(DcMotorEx.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);

        //setup
        telemetry.setAutoClear(false);
        // initialize hardware


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {
            moveForward_wDistance_wGyro(60,0.3);
            sleep(2000);

            moveBackwards_wDistance_wGyro(60,0.3);
            sleep(2000);

            //turn("right", 90);
            //sleep(2000);

            //turn("left", 90);
            //sleep(2000);
            //eesha is epicer than aarush NO
//



            sleep(9000000);

        }





    }

    public void initialize(HardwareMap hardwareMap){

        //setup
        telemetry.setAutoClear(true);

        // map imu
        imu = hardwareMap.get(BHI260IMU.class,"imu");
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,RevHubOrientationOnRobot.UsbFacingDirection.FORWARD )
        );
        imu.initialize(myIMUParameters);
        imu.resetYaw();
        // Start imu initialization

        telemetry.addData("initialize:Gyro Status", "Initialized");
        telemetry.update();
        // map motors
        bl = hardwareMap.get(DcMotorEx.class, "LB");
        fl = hardwareMap.get(DcMotor.class, "LF");
        fr = hardwareMap.get(DcMotor.class, "RF");
        br = hardwareMap.get(DcMotor.class, "RB");


    }

    public double PID_Turn (double targetAngle, double currentAngle, String minPower) {
        double sign = 1;
        double power = (targetAngle - currentAngle) * 0.015; // was 0.05
        if (minPower.equalsIgnoreCase("on")&& (power != 0)) {
            sign = Math.signum(power);
            power = Math.max(Math.abs(power), 0.1);
            power = power*sign;
        }
        return power;
    }

    public double PID_FB (double targetEC, double currentEC,double MPower)
    {
        double power = (targetEC -currentEC)*0.0001;
        if (power < 0)
        {
            power = 0;
        }
        //else if (power < 0.1)
        //{
           // power = 0.1;
        //}
        if (power > MPower){
            power = MPower;
        }

        return power;
    }

    public void turn(String direction, double targetAngle)
    {
        imu.resetYaw();
        if (direction.equalsIgnoreCase("right")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            telemetry.addData("turnRight:Init Angle",myRobotOrientation.thirdAngle);
            telemetry.update();
            while (Math.abs(targetAngle-Math.abs(myRobotOrientation.thirdAngle))>0.1) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double power = PID_Turn(targetAngle, Math.abs(myRobotOrientation.thirdAngle),"on");
                bl.setPower(power);
                fl.setPower(power);
                fr.setPower(-power);
                br.setPower(-power);
            }
            telemetry.addData("turnRight:Curr Angle",myRobotOrientation.thirdAngle);
            telemetry.update();
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
        } else if(direction.equalsIgnoreCase("left")){
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            telemetry.addData("turnLeft:Init Angle",myRobotOrientation.thirdAngle);
            telemetry.update();

            while (Math.abs(targetAngle-Math.abs(myRobotOrientation.thirdAngle))>0.1) {
                myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
                double power = PID_Turn(targetAngle, Math.abs(myRobotOrientation.thirdAngle),"on");
                bl.setPower(-power);
                fl.setPower(-power);
                fr.setPower(power);
                br.setPower(power);
            }
            telemetry.addData("turnLeft:Curr Angle",myRobotOrientation.thirdAngle);
            telemetry.update();
            bl.setPower(0);
            fl.setPower(0);
            fr.setPower(0);
            br.setPower(0);

        }
        imu.resetYaw(); // [ AARUSH ]

    }


    // Set motor directions
    public void setMotorOrientation()
    {
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);

    }

    //reset encoder counts
    public void resetMotorEncoderCounts()
    {
        // Reset encoder counts kept by motors
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode((DcMotor.RunMode.STOP_AND_RESET_ENCODER));
        telemetry.addData("Encoder", "Count Reset");  // telemetry: Mode Waiting
        telemetry.update();

    }

    public int moveForward_wDistance_wGyro(double DistanceAbsIn,double Mpower)
    {
        imu.resetYaw();
        double currZAngle = 0;
        double prevZAngle = 0;
        int currEncoderCount = 0;
        int prevEncoderCount = 0;
        double currErrEC = 0;
        double encoderAbsCounts = ENC2DIST*DistanceAbsIn;

        telemetry.addData("EC Target", encoderAbsCounts);
        telemetry.update();

        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // move forward
        timer.reset();
        long startTime = System.nanoTime();
        long endTime = 0;
        while (br.getCurrentPosition() > -encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            currZAngle = myRobotOrientation.thirdAngle;
            double correction = PID_Turn(0,currZAngle,"off");
            currEncoderCount = br.getCurrentPosition();

            endTime = System.nanoTime();
            long duration = endTime - startTime;
            double duration_msec = duration/1000000.0;
            double power = PID_FB(encoderAbsCounts,Math.abs(currEncoderCount),Mpower);
            startTime = System.nanoTime();

            bl.setPower(power-correction);
            fl.setPower(power-correction);
            fr.setPower(power+correction);
            br.setPower(power+correction);
            telemetry.clear();
            telemetry.addData("odometry wheel", br.getCurrentPosition());
            telemetry.addData("power", power);
            telemetry.addData("correction:", correction);
            telemetry.addData("duration_msec", duration_msec);
            telemetry.update();

            // quick correct for angle if it is greater than 10 [Aarush]
            double absError_angle = Math.abs(currZAngle);
            if (absError_angle > 10 && false)
            {
                turnToZeroAngle();
            }
//            // identify if you are stuck [Aarush]
//            double flagStuck = amIStuck_FB(encoderAbsCounts, currEncoderCount, prevEncoderCount);
//            if (flagStuck==1)
//                break;
//            else if (flagStuck==0)
//                prevEncoderCount = currEncoderCount;
//            else
//                // nothing to do
//            idle();
        }
//        turnToZeroAngle();

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = br.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currZAngle = myRobotOrientation.thirdAngle;
        telemetry.addData("fw:currEncoderCount", currEncoderCount);
        telemetry.addData("fw:currZAngle", currZAngle);
        telemetry.update();
        return (currEncoderCount);
    }

    public void setMotorToZeroPower()
    {
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    //move backwards with gyro correction
    public int moveBackwards_wDistance_wGyro(double DistanceAbsIn,double Mpower)
    {
        imu.resetYaw();
        double currZAngle = 0;
        double prevZAngle = 0;
        int currEncoderCount = 0;
        int prevEncoderCount = 0;
        double currErrEC = 0;

        double encoderAbsCounts = ENC2DIST *DistanceAbsIn;
        telemetry.addData("Im here",currZAngle);

        // Resetting encoder counts
        resetMotorEncoderCounts();

        // Setting motor to run in runToPosition
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // move backward
        timer.reset();
        while(br.getCurrentPosition() < encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            currZAngle = myRobotOrientation.thirdAngle;
            double correction = PID_Turn(0,currZAngle,"off");
            currEncoderCount = br.getCurrentPosition();
            double power = PID_FB(encoderAbsCounts,Math.abs(currEncoderCount),Mpower);

            bl.setPower(-power-correction);
            fl.setPower(-power-correction);
            fr.setPower(-power+correction);
            br.setPower(-power+correction);
            telemetry.clear();
            telemetry.addData("bw:power", power);
            telemetry.addData("bw:correction", correction);
            telemetry.update();

            // quick correct for angle if it is greater than 10 [Aarush]
            double absError_angle = Math.abs(currZAngle);
            if (absError_angle > 10 && false)
            {
                turnToZeroAngle();
            }
//            // identify if you are stuck [Aarush]
//            double flagStuck = amIStuck_FB(encoderAbsCounts, currEncoderCount, prevEncoderCount);
//            if (flagStuck==1)
//                break;
//            else if (flagStuck==0)
//                prevEncoderCount = currEncoderCount;
//            else
//                // nothing to do


        }


        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = br.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currZAngle = myRobotOrientation.thirdAngle;
        telemetry.addData("bw:currEncoderCount", currEncoderCount);
        telemetry.addData("bw:currZAngle", currZAngle);
        telemetry.update();
        return (currEncoderCount);
    }

    public int moveSideways_wCorrection(String direction, int DistanceAbsIn, double motorAbsPower)
    {
        turnToZeroAngle();
        int currEncoderCount = 0;
        double encoderAbsCounts = ENC2DIST_SIDEWAYS*DistanceAbsIn; //2000/42
        setMotorOrientation();
        // Resetting encoder counts
        resetMotorEncoderCounts();
        telemetry.addData("Encoder count target",encoderAbsCounts);

        // Setting motor to run in runToPosition\
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Wait for robot to finish this movement
        double refEC = 0;
        while (refEC < encoderAbsCounts) {

            double frEC = fr.getCurrentPosition();
            double blEC = bl.getCurrentPosition();
            double flEC = fl.getCurrentPosition();
            double brEC = br.getCurrentPosition();
            double frCorr = 1;
            double blCorr = 1;
            double flCorr = 1;
            double brCorr = 1;
            if (frEC != 0 ) {
                frEC = Math.abs(frEC);
                refEC = frEC;
                blEC = Math.abs(blEC);
                refEC = Math.min(refEC, blEC);
                flEC = Math.abs(flEC);
                refEC = Math.min(refEC, flEC);
                brEC = Math.abs(brEC);
                refEC = Math.min(refEC, brEC);
                if (refEC == 0) {
                    refEC = 1;
                }
                frCorr = refEC / frEC;
                blCorr = refEC / blEC;
                flCorr = refEC / flEC;
                brCorr = refEC / brEC;
            }
            double flPow = -motorAbsPower * flCorr;
            double blPow = motorAbsPower * blCorr;
            double frPow = motorAbsPower * frCorr;
            double brPow = -motorAbsPower * brCorr;

            if (direction.equalsIgnoreCase("left")) {
                fl.setPower(flPow);
                bl.setPower(blPow);
                fr.setPower(frPow);
                br.setPower(brPow);
            }
            else if (direction.equalsIgnoreCase("right")) {
                fl.setPower(-flPow);
                bl.setPower(-blPow);
                fr.setPower(-frPow);
                br.setPower(-brPow);
            }
            idle();
        }
        turnToZeroAngle();

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        telemetry.addData("sideways:currEncoderCount (final)", currEncoderCount);
        telemetry.update();
        return (currEncoderCount);
    }

    public void turnToZeroAngle()
    {
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        double targetAngle = myRobotOrientation.thirdAngle;
        if (targetAngle > 0)
        {
            telemetry.addData("turnToZeroAngle:targetAngle",targetAngle);
            telemetry.update();
            turn("right", Math.abs(targetAngle));
        }
        else if (targetAngle < 0)
        {
            telemetry.addData("turnToZeroAngle:targetAngle",targetAngle);
            telemetry.update();
            turn("left", Math.abs(targetAngle));
        }
        imu.resetYaw();
    }


}




