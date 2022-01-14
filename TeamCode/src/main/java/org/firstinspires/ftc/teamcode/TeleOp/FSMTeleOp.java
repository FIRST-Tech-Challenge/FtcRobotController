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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp

public class FSMTeleOp extends OpMode {
    
    // Declare OpMode members.
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    private DcMotor bucket;
    private DcMotor linearSlide;
    private DcMotor carouselTurner;
    private DcMotor bucketTurner;
    
    public enum LinearSlideState {
    
        LINEAR_SLIDE_START,
        LINEAR_SLIDE_EXTEND,
        LINEAR_SLIDE_BUCKET,
        LINEAR_SLIDE_RELEASE,
        LINEAR_SLIDE_RETRACT
    
    
    }
    
    LinearSlideState linearSlideState = LinearSlideState.LINEAR_SLIDE_START;
    
    int LINEAR_SLIDE_HIGH = -150;
    int LINEAR_SLIDE_LOW = -15;
    
    int BUCKET_TURNER_HIGH = -30;
    int BUCKET_TURNER_LOW = -20;
    
    double BUCKET_IDLE = 0;
    double BUCKET_RELEASE = 1;
    
    double RELEASE_TIME = 1000;
    
    ElapsedTime releaseTimer = new ElapsedTime();
    
    
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        
        initRobot();

        
                    
        
        

        
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        
        //linearSlide.setTargetPosition(-150);
        //linearSlide.setPower(-1);
        
        
        telemetry.addData("Linear Slide: ", linearSlide.getCurrentPosition() );
        telemetry.addData("Bucket Turner: ", bucketTurner.getCurrentPosition() );
        telemetry.update();
        
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        
        
        
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        
        telemetry.addData("Automation Stage", linearSlideState );
        telemetry.update();
                    
        switch (linearSlideState) {
            case LINEAR_SLIDE_START:
                    
                    linearSlide.setPower(gamepad2.right_stick_y);
                    
                    bucketTurner.setPower(gamepad2.left_stick_y * 0.5);
                    
                    
                    if (gamepad2.dpad_up) {
                        
                        bucket.setPower(1);
                        
                    } else if (gamepad2.dpad_down) {
                        
                        bucket.setPower(-0.75);
                        
                    } else if (gamepad2.dpad_right) {
                        
                        bucket.setPower(0);
                        
                    }
                    
                    if (gamepad2.y) {
                        
                        carouselTurner.setPower(1);
                    
                    } else if (gamepad2.a) {
                        
                        carouselTurner.setPower(-1);
                        
                    } else {
                        
                        carouselTurner.setPower(0);
                        
                    }
                    
                    
                    double y = -gamepad1.left_stick_y; // Remember, this is reversed!
                    double x = gamepad1.left_stick_x * 1.3; // Counteract imperfect strafing
                    double rx = gamepad1.right_stick_x;
                    // Denominator is the largest motor power (absolute value) or 1
                    // This ensures all the powers maintain the same ratio, but only when
                    // at least one is out of the range [-1, 1]
                    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                    double frontLeftPower = (y + x + rx) / denominator;
                    double backLeftPower = (y - x + rx) / denominator;
                    double frontRightPower = (y - x - rx) / denominator;
                    double backRightPower = (y + x - rx) / denominator;
                    
                    frontLeft.setPower(frontLeftPower * 0.75);
                    backLeft.setPower(backLeftPower * 0.75);
                    frontRight.setPower(frontRightPower * 0.75);
                    backRight.setPower(backRightPower * 0.75);
            
                    if (gamepad1.x) {
                    linearSlide.setTargetPosition(LINEAR_SLIDE_HIGH);
                    
                    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    linearSlide.setPower(1);
                    
                    while (linearSlide.isBusy()) {
                        
                        
                    }
                    
                    linearSlideState = LinearSlideState.LINEAR_SLIDE_EXTEND;
                    
                    telemetry.addData("Automation Stage", linearSlideState );
                    telemetry.update();
                    
                    }
                
                break;
                
            case LINEAR_SLIDE_EXTEND:
                
                if (Math.abs(linearSlide.getCurrentPosition() - LINEAR_SLIDE_HIGH) < 5) {
                
                    bucketTurner.setTargetPosition(BUCKET_TURNER_HIGH);
                    
                    bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    bucketTurner.setPower(1);
                    
                    while (bucketTurner.isBusy()) {
                        
                        
                    }
                   
                    linearSlideState = LinearSlideState.LINEAR_SLIDE_BUCKET;

                }
                break;
           
            case LINEAR_SLIDE_BUCKET:
                
                //if (Math.abs(bucketTurner.getCurrentPosition() - BUCKET_TURNER_HIGH) < 5) {
                
                    bucket.setTargetPosition(-750);
                    
                    bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    bucket.setPower(1);
                    
                    while (bucket.isBusy()) {
                        
                        
                    }
                    
                    linearSlideState = LinearSlideState.LINEAR_SLIDE_RELEASE;
                    
                    releaseTimer.reset();
                
                //}
                break;
            
            case LINEAR_SLIDE_RELEASE:
                
                //if (releaseTimer.seconds() >= RELEASE_TIME) {
                    
                    bucket.setPower(BUCKET_IDLE);
                    
                    bucketTurner.setTargetPosition(BUCKET_TURNER_LOW);
                    linearSlide.setTargetPosition(LINEAR_SLIDE_LOW);
                    
                    bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    
                    bucketTurner.setPower(-1);
                    linearSlide.setPower(-1);

                    
                    while (bucketTurner.isBusy() || linearSlide.isBusy()) {
                        
                        
                    }
                    
                    linearSlideState = LinearSlideState.LINEAR_SLIDE_RETRACT;
                    
                //}
                
                break;
                
            case LINEAR_SLIDE_RETRACT:
            
                //if (Math.abs(linearSlide.getCurrentPosition() - LINEAR_SLIDE_HIGH) < 10) {
                    
                    linearSlide.setPower(0);
                    bucketTurner.setPower(0);
                    
                    linearSlideState = LinearSlideState.LINEAR_SLIDE_START;
                    
                //}
                break;
            
            default:
            
                linearSlideState = LinearSlideState.LINEAR_SLIDE_START;

        
        
        
 
        
        }
        
        
    
        if (gamepad1.y && linearSlideState != LinearSlideState.LINEAR_SLIDE_START) {
        
            linearSlideState = LinearSlideState.LINEAR_SLIDE_START;
            
        }
        
        
        linearSlide.setPower(gamepad2.right_stick_y);
            
        bucketTurner.setPower(gamepad2.left_stick_y * 0.5);
            
            
            if (gamepad2.dpad_up) {
                
                bucket.setPower(1);
                
            } else if (gamepad2.dpad_down) {
                
                bucket.setPower(-0.75);
                
            } else if (gamepad2.dpad_right) {
                
                bucket.setPower(0);
                
            }
            
            if (gamepad2.y) {
                
                carouselTurner.setPower(1);
            
            } else if (gamepad2.a) {
                
                carouselTurner.setPower(-1);
                
            } else {
                
                carouselTurner.setPower(0);
                
            }
            
            
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.3; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            
            frontLeft.setPower(frontLeftPower * 0.75);
            backLeft.setPower(backLeftPower * 0.75);
            frontRight.setPower(frontRightPower * 0.75);
            backRight.setPower(backRightPower * 0.75);
 
            
            telemetry.addData("Status", "Running");
            telemetry.update();
            
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }


    private void initRobot() {
        
        frontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"FrontRight");
        backLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        backRight = hardwareMap.get(DcMotor.class,"BackRight");
        
        bucket = hardwareMap.get(DcMotor.class,"Bucket");
        bucketTurner = hardwareMap.get(DcMotor.class, "BucketTurner" );
        linearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        carouselTurner = hardwareMap.get(DcMotor.class, "CarouselTurner" );
        
        frontLeft.setDirection(DcMotor.Direction.FORWARD) ;
        frontRight.setDirection(DcMotor.Direction.REVERSE) ;
        backLeft.setDirection(DcMotor.Direction.FORWARD) ;
        backRight.setDirection(DcMotor.Direction.REVERSE) ;
        
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        
        bucket.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        carouselTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;

        /*
        bucket.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
        bucketTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION) ;
        carouselTurner.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */
        

        linearSlide.setDirection(DcMotor.Direction.FORWARD) ;
        
        
        bucket.setDirection(DcMotor.Direction.FORWARD) ;
 

        bucketTurner.setDirection(DcMotor.Direction.REVERSE) ;
 
        
        carouselTurner.setDirection(DcMotor.Direction.FORWARD) ;

        
        bucketTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        releaseTimer.reset();
        
        
    }
}
