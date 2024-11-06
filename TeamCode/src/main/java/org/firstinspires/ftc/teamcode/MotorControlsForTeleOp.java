package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.robotcore.hardware.SoundPlayer;

@TeleOp

public class MotorControlsForTeleOp extends LinearOpMode {
     Gamepad stupidGamepad = new Gamepad();
     Gamepad RsFault = new Gamepad();
     
     public int armThing = 0;
    @Override
    
    //Defines the motor
    public void runOpMode() {
        //Motors for Wheels
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft"); //Port 0
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight"); //Port 1
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft"); 
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        //Motors for Linear Slide
        //DcMotor leftSlide = hardwareMap.get(DcMotor.class, "leftSlide"); //Slot 0
        
       DcMotor armOne = hardwareMap.get(DcMotor.class,"armOne");
       DcMotor armTwo = hardwareMap.get(DcMotor.class,"armTwo");
        Servo clawRotate = hardwareMap.get(Servo.class,"clawRotate");
        Servo clawClamp = hardwareMap.get(Servo.class,"clawClamp");

        armOne.setTargetPosition(0);
        armTwo.setTargetPosition(0);
        armOne.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armTwo.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armOne.setPower(.65);
        armTwo.setPower(.65);
        
        double wrist = .4;

        
        
        /*
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters(/*
            new RevHUBOrientationOnRobot (
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
                )
            );
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);*/
        waitForStart();
        
        if (isStopRequested()) return;
        
        while (opModeIsActive()) {
         stupidGamepad.copy(gamepad1);
         RsFault.copy(gamepad2);
        //Makes the robot go move I hope. Prob not going to work
         float y = -stupidGamepad.left_stick_y;
         float x = stupidGamepad.left_stick_x;
         
         if (stupidGamepad.dpad_left) {
             x = -0.25f;
         }
         if (stupidGamepad.dpad_right) {
             x = 0.25f;
         }
         if (stupidGamepad.dpad_up) {
             y = 0.25f;
         }
         if (stupidGamepad.dpad_down) {
             y = -0.25f;
         }
         
        
         boolean rotateRight = stupidGamepad.right_bumper;
         boolean rotateLeft = stupidGamepad.left_bumper;
         boolean honk = stupidGamepad.left_stick_button;
         
         //For up-movement of linear slide
         boolean verticalUp = RsFault.dpad_up;
         //For down-movement of linear slide
         boolean verticalDown = RsFault.dpad_down;
         double armClose = RsFault.right_trigger;
         double armOpen = RsFault.left_trigger;
         double clawPosX = RsFault.left_stick_x;
         double clawPosY = -RsFault.left_stick_y;
         
         boolean clawOpen = RsFault.left_bumper;
         boolean clawClose = RsFault.right_bumper;
         double wheelCPR = 423.2116; //Counts per revolution
         double linearCPR = 72.1; 


              
         double fl = (y+x);
         double fr = (x-y);
         double bl = (y-x);
         double br = (-y-x);
           if (rotateRight) {
            fl = 0.85;
            fr = 0.85;
            bl = 0.85;
            br = 0.85;
         }
         if (rotateLeft) {
             fl = -0.85;
             fr = -0.85;
             bl = -0.85;
             br = -0.85;
         }
        if (rotateRight && rotateLeft == false) {
            if (x == 0 && y == 0) {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);   
            }
        }
        //stops it from going greater than 1/-1
         double maxNumber = Math.max(Math.abs(x)+Math.abs(y),1);
         //powers the motor for wheels
        frontLeft.setPower(fl/maxNumber*0.5);
        frontRight.setPower(fr/maxNumber*0.5);
        backLeft.setPower(bl/maxNumber*0.5);
        backRight.setPower(br/maxNumber*0.5); 
        
        //temp code
        
        /*
        if (verticalUp) {
            leftSlide.setPower(0.5);
        }                       //Prob works
        if (verticalDown) {
            leftSlide.setPower(-0.5);
        }
            if (verticalUp && verticalDown == false) {
            leftSlide.setPower(0);
        }
        */
        //Arm Code I USED TRIGGERS LETS GOOO
        // double armPower= armOpen-armClose;
        // armOne.setPower(armPower*0.65);
        // // armOne.setPower(-armOpen);
        // // armTwo.setPower(-armOpen);
        // armTwo.setPower(armPower*0.65);
        //lowers by tenth of a second
           /* if (fuRoshan > 0) {
            armOne.setPower(-0.5);
            armTwo.setPower(-0.5);
            sleep(1);
            armOne.setPower(-0.3);
            armTwo.setPower(-0.3);
            sleep(1);
            armOne.setPower(-0.1);
            armTwo.setPower(-0.1);
            sleep(1);
            armOne.setPower(-0.05);
            armTwo.setPower(-0.05);
            sleep(1);
            armOne.setPower(-0.025);
            armTwo.setPower(-0.025);
            sleep(1);
            armOne.setPower(0);
            armTwo.setPower(0);
            sleep(1);
            armTwo.setPower(0.025);
            armOne.setPower(0.025);
            
            }*/
            
        if (RsFault.dpad_up) {
            armThing -= 5;
        }
        if (RsFault.dpad_down) {
            armThing += 5;
        }
        if (RsFault.dpad_right) {
            armThing--;
        }
        if (RsFault.dpad_left) {
            armThing++;
        }
        if (armThing <= -126) {
            armThing = -126;
        }
        if (armThing >= -5) {
            armThing = -5;
        }
        
        
        if (RsFault.cross) {
            armThing = -7;
        }
        if (RsFault.triangle) {
            armThing = -126;            
        }
        if (RsFault.square) {
            armThing = -72;
        }

        // if (Math.abs(armOne.getCurrentPosition() - armThing) <= 2) {
        //     armOne.setTargetPosition(armOne.getCurrentPosition());
        //     armTwo.setTargetPosition(armTwo.getCurrentPosition());
        // } else {
        //     armOne.setTargetPosition(armThing);
        //     armTwo.setTargetPosition(armThing);
        // }
        
        armOne.setTargetPosition(armThing);
        armTwo.setTargetPosition(armThing);
            
        telemetry.addData("Arm One: ", armOne.getCurrentPosition());
        telemetry.addData("Arm Two: ", armTwo.getCurrentPosition());

       
       if (clawOpen) {
           clawClamp.setPosition(1);
       }
       if (clawClose) {
           clawClamp.setPosition(0);
       }
       
       if (RsFault.left_trigger >= .9) {
           wrist -= .05;
       }
       if (RsFault.right_trigger >= .9) {
           wrist += .05;
       }
       if (wrist >= 1) {
           wrist = 1;
       }
       if (wrist <= 0) {
           wrist = 0;
       }
       if (RsFault.b) {
           wrist = .4;
       }
       clawRotate.setPosition(wrist);
       
    //     double clawPosition = clawClamp.getPosition();
    //     if (clawPosY > 0) {
    //         clawRotate.setPosition(0.5);
    //     } 
    //     if (clawPosY < 0) {
    //         clawRotate.setPosition(0);
    //     }
    //     if (clawPosX < 0) {
    //         clawRotate.setPosition(0.25);
    //     }
    //     if (clawPosX > 0) {
    //         clawRotate.setPosition(0.75);
    //     }
    // double clawMove = 0;
    //     //more if statements YAY!!!
    //     int clawTest = 0;
    //     if (clawOpen) {
    //         clawClamp.setPosition(0);
    //         clawTest = 1;
    //         telemetry.addData("Actually Reads Input: ", clawTest);

    //     }
    //     if (clawClose) {
    //         clawClamp.setPosition(1);
    //         clawTest = 2;
    //         telemetry.addData("Reads Input: ", clawTest);
    //     }

         
  
        /*double botHeading = imu.getAngularOrientation().firstAngle;
         
         double triX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
         double triY = x * Math.sin(botHeading) + y * Math.cos(botHeading);
         telemetry.addData("TriX: ",triX);
         telemetry.addData("TriY: ",triY);
         telemetry.addData("botHeading: ",botHeading);*/
         //fl is front left, br is back right, etc.
        
         /*
         double fl = (triY+triX);
         double fr = (triX-triY);
         double bl = (triY-triX);
         double br = (-triY-triX);
         */
        //stops it from going greater than 1/-1
        //  maxNumber = Math.max(Math.abs(x)+Math.abs(y),1);
        //  //powers the motor for wheels
        // frontLeft.setPower(fl/maxNumber);
        // frontRight.setPower(fr/maxNumber);
        // backLeft.setPower(bl/maxNumber);
        // backRight.setPower(br/maxNumber); 
        telemetry.update();
    
        
       /*int soundID = hardwareMap.apContext.getResources().getIdentifier("horn", "raw", hardwareMap.appContext.getPackageName());
         if (soundID != 0 && honk == True) {
             SoundPlayer.getInstance().startPlaying(hardwareMap.appContext, soundID);
         }*/

        
        
        
        
        
        
        }
    }
}



