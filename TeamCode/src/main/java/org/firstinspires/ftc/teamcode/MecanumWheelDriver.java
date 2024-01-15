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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

/**     put the following code inside runOpMode()

            MecanumWheelDriver drive = new MecanumWheelDriver(hardwareMap);

        if you are going to run using encoders then include this as well

            drive.RunWithEncoders(true);

        if you are going to run anything on another thread then add this
        at the top of runOpMode()

            ExecutorService pool = Executors.newFixedThreadPool(2);

        to call a function use this format

            drive.[function name]([var 1], [var 2], ...);

        info for individual functions are included at the top of the function


        All degrees inputs are in this format

            0
         90   -90
           180
 */

public class MecanumWheelDriver implements Runnable {

    private int Angle_Degrees;
    private double inches;
    public double speed;
    private double agl_frwd;

    private int Degrees;
    private double MaxSpeed;
    private boolean toDegree;

    private boolean isMove; // true = moveInches(), false = rotate()
    public boolean moveDone = true;

    public boolean stop;

    private final double COUNTS_PER_REVOLUTION = 560;
    private final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    //final double ROBOT_DIAMETER_INCHES = 23;
    final double COUNTS_PER_INCH = COUNTS_PER_REVOLUTION / (WHEEL_DIAMETER_INCHES * Math.PI);
    //final double COUNTS_PER_DEGREE = ((ROBOT_DIAMETER_INCHES * 3.14159) / 360) * COUNTS_PER_INCH;

    private final double turnAccrate = 1;
    private double speedmin = 0.1;
    private int rampDownAngl = 55;
    private final double selfCorrectAngle = 15;
    boolean selfcorrect = true;

    int LF_RBtarget;
    int RF_LBtarget;

    int leftfrontStartPos;
    int rightfrontStartPos;
    int leftbackStartPos;
    int rightbackStartPos;

    RobotHardware H;
    LinearOpMode opMode;

    MecanumWheelDriver(RobotHardware H, LinearOpMode opMode) {

        this.H = H;
        this.opMode = opMode;

    }

    public void run() {
        /**
         * to run a function use either
         *    drive.setrotate(int, double, boolean);
         * or
         *    drive.setMoveInches(int, double, double, double);
         * then run this to start it
         *    pool.execute(drive);
         *
         * this is only for MoveInches() and Rotate() functions
         */

        if (isMove) {

            MoveInches(/*Angle_Degrees, inches, speed, agl_frwd*/);

        } else {

            rotate(/*Degrees, MaxSpeed, toDegree*/);

        }

        moveDone = true;
    }

    void move(double Angle_Degrees, double Speed, double Rotate) {

        /**Angle_Degrees, the angle relative to the robot that it should move
         * Speed, the speed at which to run the motors
         * Rotate, used to rotate the robot. can be set while moving
         *
         * move() will run motors until another function that changes motor speed is called
         * you can use stop() to stop the motors
         *
         * if you want to move backwards you either set speed negative or Angle_degrees to 180
         */

        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;

        double rTotal = Speed + Math.abs(Rotate);
        double Angle = Math.toRadians(Angle_Degrees + 45);
        double cosAngle = Math.cos(Angle);
        double sinAngle = Math.sin(Angle);
        double LF_RB;  //leftfront and rightback motors
        double RF_LB;  //rightfront and leftback motors
        double multiplier;

        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
            multiplier = 1/Math.abs(cosAngle);
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        } else {
            multiplier = 1/Math.abs(sinAngle);
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        }

        leftfrontPower    = LF_RB * Speed + Rotate;
        rightfrontPower   = RF_LB * Speed - Rotate;
        leftbackPower     = RF_LB * Speed + Rotate;
        rightbackPower    = LF_RB * Speed - Rotate;

        if (Math.abs(rTotal) > 1) {
            leftfrontPower    = leftfrontPower/rTotal;
            rightfrontPower   = rightfrontPower/rTotal;
            leftbackPower     = leftbackPower/rTotal;
            rightbackPower    = rightbackPower/rTotal;
        }

        H.driveMotor[0].  setPower(leftfrontPower);
        H.driveMotor[1]. setPower(rightfrontPower);
        H.driveMotor[2].  setPower(rightbackPower);
        H.driveMotor[3].   setPower(leftbackPower);
    }

    void moveWithGyro(double Angle_Degrees, double Speed, double agl_frwd) {

        /**Angle_Degrees, the angle relative to the robot that it should move
         * Speed, the speed at which to run the motors
         * Rotate, used to rotate the robot. can be set while moving
         *
         * move() will run motors until another function that changes motor speed is called
         * you can use stop() to stop the motors
         *
         * if you want to move backwards you either set speed negative or Angle_degrees to 180
         */

        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;

        double Angle = Math.toRadians(Angle_Degrees + 45);
        double cosAngle = Math.cos(Angle);
        double sinAngle = Math.sin(Angle);
        double LF_RB;  //leftfront and rightback motors
        double RF_LB;  //rightfront and leftback motors
        double multiplier;
        double offset;

        if (agl_frwd == -1) {
            agl_frwd = H.getheading();
        }

        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
            multiplier = 1/Math.abs(cosAngle);
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        } else {
            multiplier = 1/Math.abs(sinAngle);
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        }

        offset = FindDegOffset(H.getheading(), agl_frwd + 180);

        leftfrontPower = LF_RB - offset / 45;
        rightfrontPower = RF_LB + offset / 45;
        leftbackPower = RF_LB - offset / 45;
        rightbackPower = LF_RB + offset / 45;

        if (LF_RB > RF_LB) {
            if (offset > 0) {
                multiplier = 1 / Math.abs(rightbackPower);
            } else {
                multiplier = 1 / Math.abs(leftfrontPower);
            }
        } else {
            if (offset > 0) {
                multiplier = 1 / Math.abs(rightfrontPower);
            } else {
                multiplier = 1 / Math.abs(leftbackPower);
            }
        }

        H.driveMotor[0].setPower(Range.clip(leftfrontPower * multiplier * Speed, -1, 1));
        H.driveMotor[1].setPower(Range.clip(rightfrontPower * multiplier * Speed, -1, 1));
        H.driveMotor[2].setPower(Range.clip(rightbackPower * multiplier * Speed, -1, 1));
        H.driveMotor[3].setPower(Range.clip(leftbackPower * multiplier * Speed, -1, 1));
    }

    void stop() {

        /**stops all the motors
         */

        stop = true;
        
        H.driveMotor[0].  setPower(0);
        H.driveMotor[1]. setPower(0);
        H.driveMotor[2].   setPower(0);
        H.driveMotor[3].  setPower(0);
    
        H.driveMotor[0].setTargetPosition(H.driveMotor[0].getCurrentPosition());
        H.driveMotor[1].setTargetPosition(H.driveMotor[1].getCurrentPosition());
        H.driveMotor[2].setTargetPosition(H.driveMotor[2].getCurrentPosition());
        H.driveMotor[3].setTargetPosition(H.driveMotor[3].getCurrentPosition());

        H.driveMotor[0].  setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[1]. setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[2].   setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[3].  setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    void setMoveInches(int Angle_Degrees, double inches, double speed, double agl_frwd) {

        this.Angle_Degrees = Angle_Degrees;
        this.inches = inches;
        this.speed = speed;
        this.agl_frwd = agl_frwd;

        isMove = true;
        moveDone = false;
        stop = false;

    }

    void changeTargetInches(double inches, boolean reletiveToStart) {

        double Angle = Math.toRadians(Angle_Degrees + 45);// - Math.PI / 4;
        double cosAngle = Math.cos(Angle);
        double sinAngle = Math.sin(Angle);
        double LF_RB;  //leftfront and rightback motors
        double RF_LB;  //rightfront and leftback motors
        double multiplier;

        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
            multiplier = 1 / Math.abs(cosAngle);
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        } else {
            multiplier = 1 / Math.abs(sinAngle);
            LF_RB = multiplier * cosAngle;
            RF_LB = multiplier * sinAngle;
        }

        LF_RBtarget = (int)(LF_RB * inches * COUNTS_PER_INCH);
        RF_LBtarget = (int)(RF_LB * inches * COUNTS_PER_INCH);

        if (reletiveToStart) {

            H.driveMotor[0].setTargetPosition(leftfrontStartPos + LF_RBtarget);
            H.driveMotor[1].setTargetPosition(rightfrontStartPos + RF_LBtarget);
            H.driveMotor[2].setTargetPosition(rightbackStartPos + LF_RBtarget);
            H.driveMotor[3].setTargetPosition(leftbackStartPos + RF_LBtarget);

        } else {

            H.driveMotor[0].setTargetPosition(H.driveMotor[0].getCurrentPosition() + LF_RBtarget);
            H.driveMotor[1].setTargetPosition(H.driveMotor[1].getCurrentPosition() + RF_LBtarget);
            H.driveMotor[2].setTargetPosition(H.driveMotor[2].getCurrentPosition() + LF_RBtarget);
            H.driveMotor[3].setTargetPosition(H.driveMotor[3].getCurrentPosition() + RF_LBtarget);

        }

    }

    void MoveInches(/*int Angle_Degrees, double inches, double speed, double agl_frwd*/) {

        /**Angle_Degrees, the angle relative to the robot that it should move
         * inches, the number of inches to move. Is not accurate when going sideways due to the mecanum wheels
         * speed, the speed at which to run the motors
         * agl_frwd, the direction the robot should face while moving
         * it will be set automatically if = -1
         * forward = 0 degrees, right = 90, left = -90, back = 180
         *
         * DO NOT set speed to a negative number
         * if you want to move backwards set Angle_degrees to 180
         */

        double Angle = Math.toRadians(Angle_Degrees + 45);// - Math.PI / 4;
        double cosAngle = Math.cos(Angle);
        double sinAngle = Math.sin(Angle);
        double LF_RB;  //leftfront and rightback motors
        double RF_LB;  //rightfront and leftback motors
        double multiplier;
        double offset;
        double leftfrontPower;
        double rightfrontPower;
        double leftbackPower;
        double rightbackPower;
        selfcorrect = true;

        if (agl_frwd == -1) {
            agl_frwd = H.getheading() - 180;
            selfcorrect = true;
        } else if (agl_frwd == -2) {
            selfcorrect = false;
        }

        if (Math.abs(cosAngle) > Math.abs(sinAngle)) {   //scale the motor's speed so that at least one of them = 1
            multiplier = 1 / Math.abs(cosAngle);
        } else {
            multiplier = 1 / Math.abs(sinAngle);
        }
        
        LF_RB = multiplier * cosAngle;
        RF_LB = multiplier * sinAngle;

        leftfrontStartPos = H.driveMotor[0].getCurrentPosition();
        rightfrontStartPos = H.driveMotor[1].getCurrentPosition();
        rightbackStartPos = H.driveMotor[2].getCurrentPosition();
        leftbackStartPos = H.driveMotor[3].getCurrentPosition();

        LF_RBtarget = (int)(LF_RB * inches * COUNTS_PER_INCH);
        RF_LBtarget = (int)(RF_LB * inches * COUNTS_PER_INCH);

        H.driveMotor[0].setTargetPosition(leftfrontStartPos + LF_RBtarget);
        H.driveMotor[1].setTargetPosition(rightfrontStartPos + RF_LBtarget);
        H.driveMotor[2].setTargetPosition(rightbackStartPos + LF_RBtarget);
        H.driveMotor[3].setTargetPosition(leftbackStartPos + RF_LBtarget);

        H.driveMotor[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.driveMotor[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.driveMotor[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.driveMotor[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (H.driveMotor[0].isBusy() && H.driveMotor[1].isBusy() && H.driveMotor[2].isBusy() && H.driveMotor[3].isBusy() && !stop && opMode.opModeIsActive()) {
            if (selfcorrect) {
                offset = FindDegOffset(H.getheading(), agl_frwd + 180);

                leftfrontPower = LF_RB * speed - offset / selfCorrectAngle;
                rightfrontPower = RF_LB * speed + offset / selfCorrectAngle;
                leftbackPower = RF_LB * speed - offset / selfCorrectAngle;
                rightbackPower = LF_RB * speed + offset / selfCorrectAngle;

                if (LF_RB > RF_LB) {
                    if (offset > 0) {
                        multiplier = 1 / Math.abs(rightbackPower);
                    } else {
                        multiplier = 1 / Math.abs(leftfrontPower);
                    }
                } else {
                    if (offset > 0) {
                        multiplier = 1 / Math.abs(rightfrontPower);
                    } else {
                        multiplier = 1 / Math.abs(leftbackPower);
                    }
                }

                H.driveMotor[0].setPower(Range.clip(leftfrontPower * multiplier * speed, -1, 1));
                H.driveMotor[1].setPower(Range.clip(rightfrontPower * multiplier * speed, -1, 1));
                H.driveMotor[2].setPower(Range.clip(rightbackPower * multiplier * speed, -1, 1));
                H.driveMotor[3].setPower(Range.clip(leftbackPower * multiplier * speed, -1, 1));

            } else {

                H.driveMotor[0].setPower(LF_RB * speed);
                H.driveMotor[1].setPower(RF_LB * speed);
                H.driveMotor[2].setPower(LF_RB * speed);
                H.driveMotor[3].setPower(RF_LB * speed);

            }

        }

        H.driveMotor[0].setPower(0);
        H.driveMotor[1].setPower(0);
        H.driveMotor[2].setPower(0);
        H.driveMotor[3].setPower(0);

        H.driveMotor[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    void setrotate(int Degrees, double MaxSpeed, boolean toDegree) {

        this.Degrees = Degrees;
        this.MaxSpeed = MaxSpeed;
        this.toDegree = toDegree;

        isMove = false;
        moveDone = false;
        stop = false;

    }

    void rotate(/*int Degrees, double MaxSpeed, boolean toDegree*/) {

        /**if toDegree is false:
         * Rotates a number of degrees relative to the robot's current angle
         * the robot will rotate in one direction until it arrives at the target degree
         * the direction is determined by whether or not the input degree is negative
         * -degree = right, +degree = left
         *
         * if toDegree is true:
         * Rotates to a degree relative to the starting angle of the robot
         * for example: rotateToDeg(-90, 1); would make the robot turn at full speed to -90 degrees (right)
         * where 0 degrees is the front of the robot when the RobotHardware is initialized
         * rotateToDeg will always find the shortest direction to turn even while running
         * it will never turn more than 180 degrees
         * while running the robot can be intercepted and will still stop at the target degrees
         *
         * Both:
         * a number of degrees before the target the robot will gradually slow down to the min speed
         * both can be set at the top of the class as rampDownAngl and speedmin or with the setRampDown() function
         * DO NOT input a negative MaxSpeed if you do the robot won't move and will be stuck in an infinite loop
         */

        double speed;
        int offset;
        double heading = H.getheading();
        double Target;
        double drect;

        if (toDegree) {
            Target = Degrees + 180;
        } else {
            Target = addDegree(heading, Degrees);
        }
        
        final double startOffset = FindDegOffset(heading, Target);
        final double clicks_per_degree = 9.2222222;
    
        leftfrontStartPos = H.driveMotor[0].getCurrentPosition();
        rightfrontStartPos = H.driveMotor[1].getCurrentPosition();
        rightbackStartPos = H.driveMotor[2].getCurrentPosition();
        leftbackStartPos = H.driveMotor[3].getCurrentPosition();
    
        offset = (int)(startOffset * clicks_per_degree);
    
        H.driveMotor[0].setTargetPosition(leftfrontStartPos + offset);
        H.driveMotor[1].setTargetPosition(rightfrontStartPos - offset);
        H.driveMotor[2].setTargetPosition(rightbackStartPos - offset);
        H.driveMotor[3].setTargetPosition(leftbackStartPos + offset);
        
        H.driveMotor[0].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.driveMotor[1].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.driveMotor[2].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        H.driveMotor[3].setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        H.driveMotor[0].setPower(MaxSpeed);
        H.driveMotor[1].setPower(MaxSpeed);
        H.driveMotor[2].setPower(MaxSpeed);
        H.driveMotor[3].setPower(MaxSpeed);
    
        while ((H.driveMotor[0].isBusy() || H.driveMotor[1].isBusy() || H.driveMotor[2].isBusy() || H.driveMotor[3].isBusy()) && !stop && opMode.opModeIsActive()) {
    
            leftfrontStartPos = H.driveMotor[0].getCurrentPosition();
            rightfrontStartPos = H.driveMotor[1].getCurrentPosition();
            rightbackStartPos = H.driveMotor[2].getCurrentPosition();
            leftbackStartPos = H.driveMotor[3].getCurrentPosition();
            
            heading = H.getheading();
    
            offset = (int)(FindDegOffset(heading, Target) * clicks_per_degree);
    
            H.driveMotor[0].setTargetPosition(leftfrontStartPos + offset);
            H.driveMotor[1].setTargetPosition(rightfrontStartPos - offset);
            H.driveMotor[2].setTargetPosition(rightbackStartPos - offset);
            H.driveMotor[3].setTargetPosition(leftbackStartPos + offset);
        
        }
    
        H.driveMotor[0].setPower(0);
        H.driveMotor[1].setPower(0);
        H.driveMotor[2].setPower(0);
        H.driveMotor[3].setPower(0);
    
        H.driveMotor[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[2].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        H.driveMotor[3].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        /*do {
            heading = H.getheading();
            offset = FindDegOffset(heading, Target);
            speed = Range.clip( (Math.exp(0.1 * Math.abs(offset))-1)/(startOffset), speedmin, MaxSpeed);
            drect = Math.signum(offset);//Range.clip(offset, -1, 1);
            
            H.driveMotor[0].setPower(speed * drect);
            H.driveMotor[1].setPower(-speed * drect);
            H.driveMotor[2].setPower(-speed * drect);
            H.driveMotor[3].setPower(speed * drect);

        } while (Math.abs(offset) > turnAccrate && !stop && opMode.opModeIsActive());

        H.driveMotor[0].setPower(0);
        H.driveMotor[1].setPower(0);
        H.driveMotor[2].setPower(0);
        H.driveMotor[3].setPower(0);

        /*
        int Lefttarget = (int)(Degrees * COUNTS_PER_DEGREE);
        int Righttarget = -((int)(Degrees * COUNTS_PER_DEGREE));

        leftfront.setTargetPosition(leftfront.getCurrentPosition() + Lefttarget);
        rightfront.setTargetPosition(rightfront.getCurrentPosition() + Righttarget);
        leftback.setTargetPosition(leftback.getCurrentPosition() + Lefttarget);
        rightback.setTargetPosition(rightback.getCurrentPosition() + Righttarget);

        leftfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightfront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftback.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightback.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftfront.setPower(speed);
        rightfront.setPower(-speed);
        leftback.setPower(speed);
        rightback.setPower(-speed);

        while (leftfront.isBusy() && rightfront.isBusy() && leftback.isBusy() && rightback.isBusy()) {
            //idle();
        }

        leftfront.setPower(0);
        rightfront.setPower(0);
        leftback.setPower(0);
        rightback.setPower(0);

        leftfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }

    void setRampDown(int ramp_Down_Angle, double minimum_Speed) {
        /**sets what angle to start slowing down in rotateToDeg() and rotate()
         * and what the minimum speed should be, the robot may not go this speed if the ramp_Down_Angle is to low
         * default: ramp_Down_Angle = 50, minimum_Speed = 0.15 set either to 0 to reset to default
         * set ramp_Down_Angle to 1 for to not ramp down
         */
        if (ramp_Down_Angle <= 0) {
            rampDownAngl = 45;
        } else {
            rampDownAngl = ramp_Down_Angle;
        }
        if (minimum_Speed <= 0) {
            speedmin = 0.25;
        } else {
            speedmin = minimum_Speed;
        }
    }

    void RunWithEncoders(boolean On) {

        /**turns on or off encoders
         * will reset encoders when turned on
         * true = on, false = off
         */

        if (On) {
            H.driveMotor[0].  setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            H.driveMotor[1]. setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            H.driveMotor[2].  setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            H.driveMotor[3].   setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            H.driveMotor[0].  setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            H.driveMotor[1]. setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            H.driveMotor[2].  setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            H.driveMotor[3].   setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else {
            H.driveMotor[0].  setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            H.driveMotor[1]. setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            H.driveMotor[2].  setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            H.driveMotor[3].   setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    double POVRotate(double Target, double speed) {

        double rotateSpeed;
        double offset;
        double heading;
        byte drect;

        heading = H.heading;
        offset = -FindDegOffset(heading, Target);
        rotateSpeed = Range.clip( Math.abs(offset / rampDownAngl), 0.19, speed);
        drect = (byte)Range.clip(offset * 100, -1, 1);

        if (Math.abs(offset) > 5) {
            return rotateSpeed * drect;
        } else {
            return 0;
        }

    }

    double addDegree(double DegCurrent, double addDeg) {

        /**adds a number of degrees to the current degree with rapping around from 360 to 0
         * returns a value between 0 and 360
         */

        double output = DegCurrent + addDeg;
        while (output < 0 || output > 360) {
            if (output >= 360) {
                output -= 360;
            } else if (output < 0) {
                output += 360;
            }
        }
        return output;
    }

    double FindDegOffset(double DegCurrent, double TargetDeg) {

        /**DegCurrent, the current degree of the robot value between 0 and 360
         * TargetDeg, the degree with which to find the offset
         * Finds the angle between current degree and the target degree
         * returns a value between -180 and 180
         * output will be negative if the target degree is right, positive if on the left
         *     x
         * -90   90
         *    180
         */

        double offset = TargetDeg - DegCurrent;
        if (offset > 180) {
            offset -= 360;
        } else if (offset < -180) {
            offset += 360;
        }
        return offset;
    }
}
