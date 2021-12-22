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

public class MecanumWheelChassis extends LinearOpMode {
    
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    private DcMotor bucket;
    private DcMotor linearSlide;
    private DcMotor carouselTurner;
    private DcMotor linkage;
    
    @Override
    public void runOpMode() {
        
        initializeRobot();
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            if (gamepad2.y) {
                
                linearSlide.setPower(0.5);
                

            } else if (gamepad2.a) {
                
                linearSlide.setPower(-0.5);
                

            } else {
        
                linearSlide.setPower(0);
                

            }
            
            if (gamepad2.dpad_up) {
                
                bucket.setPower(1);
                
            } else if (gamepad2.dpad_down) {
                
                bucket.setPower(-1);
                
            } else {
                
                bucket.setPower(0);
                
            }
            
            linearSlide.setPower(gamepad2.right_stick_y);
            
            if (gamepad2.dpad_right) {
            
                linkage.setPower(0.5);
                
            } else if (gamepad2.dpad_left) {
                
                linkage.setPower(-0.5);
                
                
            } else {
                
                linkage.setPower(0);
                
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

            frontLeft.setPower(frontLeftPower * 0.5);
            backLeft.setPower(backLeftPower * 0.5);
            frontRight.setPower(frontRightPower * 0.5);
            backRight.setPower(backRightPower * 0.5);
 
            
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
    
    
    private void initializeRobot() {
        
        frontLeft = hardwareMap.get(DcMotor.class,"FrontLeft");
        frontRight = hardwareMap.get(DcMotor.class,"FrontRight");
        backLeft = hardwareMap.get(DcMotor.class,"BackLeft");
        backRight = hardwareMap.get(DcMotor.class,"BackRight");
        
        bucket = hardwareMap.get(DcMotor.class,"Bucket");
        linearSlide = hardwareMap.get(DcMotor.class, "LinearSlide" );
        linkage = hardwareMap.get(DcMotor.class, "Linkage" );
        carouselTurner = hardwareMap.get(DcMotor.class, "CarouselTurner" );
        
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
       
        linkage.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }
}
