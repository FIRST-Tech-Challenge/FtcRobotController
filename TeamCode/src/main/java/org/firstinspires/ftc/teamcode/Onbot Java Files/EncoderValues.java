/*
Copyright 2022 FIRST Tech Challenge Team 10820

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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
@TeleOp (name = "Debugging: Sensor Values", group = "Debugging")

public class EncoderValues extends LinearOpMode {
    
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    
    private DcMotorEx armMotor;
    private DcMotorEx bucketTurner;
    
    private CRServo cappingServo;

    private DistanceSensor freightDistance;
    private DistanceSensor leftDistance;
    private DistanceSensor rightDistance;
    private DistanceSensor carouselDistance;
    
    private ColorSensor frontColor;
    private ColorSensor backColor;
    
    @Override
    public void runOpMode() {
        
        frontLeft = (DcMotor)hardwareMap.get(DcMotor.class, "FrontLeft");
        frontRight = (DcMotor)hardwareMap.get(DcMotor.class, "FrontRight");
        backLeft = (DcMotor)hardwareMap.get(DcMotor.class, "BackLeft");
        backRight = (DcMotor)hardwareMap.get(DcMotor.class, "BackRight");
        
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            
        armMotor = hardwareMap.get(DcMotorEx.class, "ArmMotor");
        bucketTurner = hardwareMap.get(DcMotorEx.class, "BucketTurner" );
        
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bucketTurner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bucketTurner.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        cappingServo = hardwareMap.get(CRServo.class, "cappingServo");

        freightDistance = hardwareMap.get(DistanceSensor.class, "FreightDistance");        
        rightDistance = hardwareMap.get(DistanceSensor.class, "RightDistance");        
        leftDistance = hardwareMap.get(DistanceSensor.class, "LeftDistance");        
        carouselDistance = hardwareMap.get(DistanceSensor.class, "CarouselDistance");        

        frontColor = hardwareMap.get(ColorSensor.class, "FrontColor");        
        backColor = hardwareMap.get(ColorSensor.class, "BackColor");        
        
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            
            telemetry.addData("frontleft position: ", frontLeft.getCurrentPosition());
            telemetry.addData("frontright position: ", frontRight.getCurrentPosition());
            telemetry.addData("backleft position: ", backLeft.getCurrentPosition());
            telemetry.addData("backright position: ", backRight.getCurrentPosition());
            
            telemetry.addData("bucket position: ", bucketTurner.getCurrentPosition());
            telemetry.addData("arm position: ", armMotor.getCurrentPosition());
            
            telemetry.addData("freight distance: ", freightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("right distance: ", rightDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("left distance: ", leftDistance.getDistance(DistanceUnit.INCH));
            telemetry.addData("carousel distance: ", carouselDistance.getDistance(DistanceUnit.INCH));
            
            telemetry.addData("front color: ", frontColor.alpha());
            telemetry.addData("back color: ", backColor.alpha());
            
            telemetry.update();
            
            if (gamepad1.y) {
                
                cappingServo.setPower(-0.5);
                
                
            } else if (gamepad1.a) {
                
                cappingServo.setPower(0.1);
            
            } else {
                
                
                //cappingServo.setPower(0);
                
            }
            
            telemetry.addData("Status", "Running");
            telemetry.update();

        }
    }
}
