package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.Telemetry;
@Autonomous(name="Auton For Right Field", group="Robot")
public class AutonForRightField extends LinearOpMode {
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor leftSlide = null;
    public DcMotor rightSlide = null;
    public DcMotor Arm = null;
        public void moveWheel(double x,double y) {
         double fl = (y+x);
         double fr = (x-y);
         double bl = (y-x);
         double br = (-y-x);
        double wheelCPR = 423.2116; //Counts per revolution
         int Flposition = frontLeft.getCurrentPosition();
         int Frposition = frontRight.getCurrentPosition();
         int Blposition = backLeft.getCurrentPosition();
         int Brposition = backRight.getCurrentPosition();
         //YAY MORE VARIABLES
         double Flrevolutions = Flposition/wheelCPR;
         double Frrevolutions = Frposition/wheelCPR;
         double Blrevolutions = Blposition/wheelCPR;
         double Brrevolutions = Brposition/wheelCPR;
        double leadMotor = Math.min(Math.min(Flrevolutions,Frrevolutions),Math.min(Blrevolutions,Brrevolutions));
         //Modifier Variables YAY
        // double linearMotor = Math.min(Lsposition,Rsposition);
         double flModifier = 1;
         double frModifier = 1;
         double blModifier = 1;
         double brModifier = 1;
            if (Flrevolutions > leadMotor) {
                flModifier = leadMotor/Flrevolutions;
            }
            if (Frrevolutions > leadMotor) {
                frModifier = leadMotor/Frrevolutions;
            }
            if (Blrevolutions > leadMotor) {
                blModifier = leadMotor/Blrevolutions;
            }
            if (Brrevolutions > leadMotor) {
                brModifier = leadMotor/Brrevolutions;
            }
        //stops it from going greater than 1/-1
         double maxNumber = Math.max(Math.abs(x)+Math.abs(y),1);
         //powers the motor for wheels
        frontLeft.setPower(fl/maxNumber);
        frontRight.setPower(fr/maxNumber);
        backLeft.setPower(bl/maxNumber);
        backRight.setPower(br/maxNumber); 
        //Should do telemetery data thingy I hate this.
        telemetry.addData("Front Left",fl/maxNumber;
        telemetry.addData("Front Right",fr/maxNumber);
        telemetry.addData("Back Left",bl/maxNumber);
        telemetry.addData("Back Right",br/maxNumber);
    }
   public void turn(boolean rotateLeft, boolean rotateRight) {
        double fl, fr, bl, br;
        double wheelCPR = 423.2116; //Counts per revolution
         int Flposition = frontLeft.getCurrentPosition();
         int Frposition = frontRight.getCurrentPosition();
         int Blposition = backLeft.getCurrentPosition();
         int Brposition = backRight.getCurrentPosition();
         //YAY MORE VARIABLES
         double Flrevolutions = Flposition/wheelCPR;
         double Frrevolutions = Frposition/wheelCPR;
         double Blrevolutions = Blposition/wheelCPR;
         double Brrevolutions = Brposition/wheelCPR;
        double leadMotor = Math.min(Math.min(Flrevolutions,Frrevolutions),Math.min(Blrevolutions,Brrevolutions));
         //Modifier Variables YAY
        // double linearMotor = Math.min(Lsposition,Rsposition);
         double flModifier = 1;
         double frModifier = 1;
         double blModifier = 1;
         double brModifier = 1;
            if (Flrevolutions > leadMotor) {
                flModifier = leadMotor/Flrevolutions;
            }
            if (Frrevolutions > leadMotor) {
                frModifier = leadMotor/Frrevolutions;
            }
            if (Blrevolutions > leadMotor) {
                blModifier = leadMotor/Blrevolutions;
            }
            if (Brrevolutions > leadMotor) {
                brModifier = leadMotor/Brrevolutions;
            }
        fl = fr = bl = br = 0;
        if (rotateRight) {
            fl = 0.5;
            fr = 0.5;
            bl = 0.5;
            br = 0.5;
         }
         if (rotateLeft) {
             fl = -0.5;
             fr = -0.5;
             bl = -0.5;
             br = -0.5;
         }
    } 
    public void linearVertical(float verticalPower) {
        if (verticalPower>0) {
            leftSlide.setPower(verticalPower);
            rightSlide.setPower(verticalPower);
        }                       //Assumes triggers to be positive values
        else if (verticalPower<0) {
            leftSlide.setPower(verticalPower);
            rightSlide.setPower(verticalPower);
        }
    }
    public void armPleaseWork(float armVertical) {
       if (armVertical > 0) {
           Arm.setPower(armVertical);
       } else if (armVertical < 0) {
           Arm.setPower(armVertical);
       }
    }
        //Code for moving in different directions. may have to reverse
        public void moveForward(int seconds) {
        //sets current time
        long durationSucks = System.currentTimeMillis();
        //stops current time
        long stop = (durationSucks + seconds*100); //Im using tenths of a seconds, not seconds btw
        while (System. currentTimeMillis() < stop) {
            //not even sure if this works or not. We can test it anyway
            moveWheel(0,0.5);
        }
    }
        public void moveBackward(int seconds) {
            //sets current time
            long durationSucks = System.currentTimeMillis();
            //stops current time
            long stop = (durationSucks + seconds*100);
            while (System. currentTimeMillis() < stop) {
                //not even sure if this works or not. We can test it anyway
                moveWheel(0,-0.5);
        }
    }
    //Hopefully strafe code works
        public void strafeRight(int seconds) {
            //sets current time
            long durationSucks = System.currentTimeMillis();
            //stops current time
            long stop = (durationSucks + seconds*100);
            while (System. currentTimeMillis() < stop) {
                //not even sure if this works or not. We can test it anyway
                moveWheel(0.5,0);
        }
    }
        public void strafeLeft(int seconds) {
            //sets current time
            long durationSucks = System.currentTimeMillis();
            //stops current time
            long stop = (durationSucks + seconds*100);
                while (System. currentTimeMillis() < stop) {
                //not even sure if this works or not. We can test it anyway
                moveWheel(-0.5,0);
        }
    }
       public void rotateLeft(int seconds) {
            //sets current time
            long durationSucks = System.currentTimeMillis();
            //stops current time
            long stop = (durationSucks + seconds*100);
                while (System. currentTimeMillis() < stop) {
                turn(true,false);
        }
    }
    public void rotateRight(int seconds) {
            //sets current time
            long durationSucks = System.currentTimeMillis();
            //stops current time
            long stop = (durationSucks + seconds*100);
                while (System. currentTimeMillis() < stop) {
                turn(false,true);
        }
    }
    
     public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft"); 
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
        DcMotor leftSlide = hardwareMap.get(DcMotor.class,"leftSlide");
        DcMotor rightSlide = hardwareMap.get(DcMotor.class,"rightSlide");
        DcMotor Arm = hardwareMap.get(DcMotor.class,"Arm");
        //Not even sure it runs
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //More code
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();
          if (opModeIsActive()) {
            moveForward(1);
            
          }
         while (opModeIsActive()) {
            //Updates the thingy
            telemetry.update();

    }
   
    }
}
