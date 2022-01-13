/*
Copyright 2021 FIRST Tech Challenge Team FTC
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this opmode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp

public class FSMTeleOp extends OpMode {
    
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
    
    public void init() {
    
        frontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"FrontRight");
        backLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        backRight = hardwareMap.get(DcMotor.class,"BackRight");
        
        frontLeft.setDirection(DcMotor.Direction.FORWARD) ;
        frontRight.setDirection(DcMotor.Direction.REVERSE) ;
        backLeft.setDirection(DcMotor.Direction.FORWARD) ;
        backRight.setDirection(DcMotor.Direction.REVERSE) ;
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER) ;
 
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER) ;

        bucket = hardwareMap.get(DcMotor.class,"Bucket");
        bucketTurner = hardwareMap.get(DcMotor.class, "BucketTurner" );
        linearSlide = hardwareMap.get(DcMotor.class, "LinearSlide");
        carouselTurner = hardwareMap.get(DcMotor.class, "CarouselTurner" );


        linearSlide.setDirection(DcMotor.Direction.FORWARD) ;
        
        
        bucket.setDirection(DcMotor.Direction.FORWARD) ;
 

        bucketTurner.setDirection(DcMotor.Direction.REVERSE) ;
 
        
        carouselTurner.setDirection(DcMotor.Direction.FORWARD) ;

        
        bucketTurner.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    
    
    }
        
        

    public void loop() {
    
        switch(linearSlideState) {
            case LinearSlideState.LINEAR_SLIDE_START:
                if (gamepad1.x) {
                
                    linearSlide.setPosition();
                    linearSlideState = LinearSlideState.LINEAR_SLIDE_EXTEND;
                
                }
                break;
                
            case LinearSlideState.LINEAR_SLIDE_EXTEND:
                
                if (Math.abs(linearSlide.getPosition() - LINEAR_SLIDE_HIGH) < 10) {
                
                    bucketTurner.setPosition(BUCKET_TURNER_HIGH);
                    linearSlideState = LinearSlideState.LINEAR_SLIDE_BUCKET;

                }
                break;
           
            case LinearSlideState.LINEAR_SLIDE_BUCKET:
                
                if (Math.abs(bucketTurner.getPosition() - BUCKET_TURNER_HIGH) < 10) {
                
                    bucket.setPower(-1);
                    
                
                
                
                }
                    
                    
        
        }
 
    
    
    
    
    }
    
    /*
    public void runOpMode() throws InterruptedException {
        
        initializeRobot();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
          
            /*
            linearSlide.setPower(gamepad2.right_stick_y * 0.5);
            
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
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
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
    }
    
    */
    
}
