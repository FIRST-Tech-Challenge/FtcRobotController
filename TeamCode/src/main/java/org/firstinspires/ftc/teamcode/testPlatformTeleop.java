package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="testPlatfromTeleop", group="Zippo")
//@Disabled

//THE KEY BELOW IS FOR USING IMAGE RECOGNITION -PULKIT
//AT5CUvf/////AAAAGaBn6TlejU79iRr5dpGz0Msa4+WbMquS0c0rHQGMURBOGIxPznOmaavjYRYfWHE/qRnpaHDvKIVV1drOmZguwKjiTVfUzVpkRgxdFzcVDsNBldxzhrcSl+bRKGlNv3zKHDfaOJioTa7uzIN/uKUzdJPX+o5PQQxRPYXBuIvAkASbZ9/MVjq5u3Jltyw3Gz9DCPVgxqcMKILOwv9FpMDMRTcgeRwk7f+pPd8f5FmB8ehr3xiDbPxydmYAkpuqQ6Mx2qiggbSlzl4uTm2JeqOP3hbej+ozcevtHKh9C4S3eKodfDUpKekBfdOuR2aer0FwrWxfAqmdOewy5Tei71lLAOgEzx+vo6OPKpSzbTh1gFzI

public class testPlatformTeleop extends OpMode{

    testPlatformHardware robot  = new testPlatformHardware();

    // Create variables for motor power
    private double flPower = 0; //left wheel
    private double frPower = 0; //right wheel
    private double blPower = 0; //front power
    private double brPower = 0; //back power
    private double liftPower = 0; // lift power

    private boolean scalePower = false;




    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        //robot.sensorColor.enableLed(false);
        double time =  System.currentTimeMillis();

    }

    @Override
    public void loop() {
//      MAIN DRIVING CONTROLS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
//        lPower = gamepad1.right_stick_y;
//        rPower = gamepad1.right_stick_y;
//        fPower = -gamepad1.right_stick_x;
//        bPower = -gamepad1.right_stick_x;
        if ((gamepad1.left_stick_y > 0.1 || gamepad1.left_stick_y < -0.1)||(gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1))
        {
            if(scalePower == true) {
                flPower = gamepad1.left_stick_y * 0.5;
                brPower = gamepad1.left_stick_y * 0.5;
                frPower = gamepad1.left_stick_y * 0.5;
                blPower = gamepad1.left_stick_y * 0.5;
            }
            else{
                flPower = gamepad1.left_stick_y * 2;
                brPower = gamepad1.left_stick_y * 2;
                frPower = gamepad1.left_stick_y * 2;
                blPower = gamepad1.left_stick_y * 2;
            }
        }
        else if ((gamepad1.left_stick_x < -0.2 || gamepad1.left_stick_x > .2) || (gamepad2.left_stick_x <-.02 || gamepad2.left_stick_x > .2))
        {
            if(scalePower == true) {
                frPower = gamepad1.left_stick_x * 0.5;
                blPower = gamepad1.left_stick_x * 0.5;
                flPower = -gamepad1.left_stick_x * 0.5;
                brPower = -gamepad1.left_stick_x * 0.5;
            }
            else{
                frPower = gamepad1.left_stick_x * 2;
                blPower = gamepad1.left_stick_x * 2;
                flPower = -gamepad1.left_stick_x * 2;
                brPower = -gamepad1.left_stick_x * 2;
            }

        }
        else
        {
            flPower = 0;
            brPower = 0;
            frPower = 0;
            blPower = 0;
        }

//        if (gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1)
//        {
//            flPower = gamepad1.left_stick_y * 2;
//            brPower = gamepad1.left_stick_y *2;
//            frPower = gamepad1.left_stick_y *2;
//            blPower = gamepad1.left_stick_y* 2;
//        }
//
//
//        else if (gamepad2.left_stick_x < -0.2 || gamepad2.left_stick_x > .2)
//        {
//            frPower = gamepad1.left_stick_x * 2;
//            blPower = gamepad1.left_stick_x * 2;
//            flPower = -gamepad1.left_stick_x * 2;
//            brPower = -gamepad1.left_stick_x * 2;
//        }
//        else
//        {
//            flPower = 0;
//            brPower = 0;
//            frPower = 0;
//            blPower = 0;
//        }

        //rotate


        if (gamepad1.right_stick_x < -0.1 || gamepad2.right_stick_x < -0.1) {
            flPower = 0.5;
            brPower = -0.5;
            frPower = -0.5;
            blPower = 0.5;
        } else if (gamepad1.right_stick_x > 0.1 || gamepad2.right_stick_x> 0.1) {
            flPower = -0.5;
            brPower = 0.5;
            frPower = 0.5;
            blPower = -0.5;
        }


        if(frPower > 1)
        {
            frPower = 1;
        }
        else if (frPower < -1)
        {
            frPower = -1;
        }

        if(flPower > 1)
        {
            flPower = 1;
        }
        else if (flPower < -1)
        {
            flPower = -1;
        }

        if(blPower > 1)
        {
            blPower = 1;
        }
        else if (blPower < -1)
        {
            blPower = -1;
        }
        if(brPower > 1)
        {
            brPower = 1;
        }
        else if (brPower < -1)
        {
            brPower = -1;
        }
//
        if (gamepad1.right_trigger > 0)
        {
            scalePower = true;
        }
        else
        {
            scalePower = false;
        }
        //BringDown hook = new BringDown();




//      SCALING POWERS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        if(scalePower)
        {
            flPower *= .7;
            brPower *= .7;
            frPower *= .7;
            blPower *= .7;
        }

//      SETTING POWERS AND POSITIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        robot.motorFrontLeft.setPower(flPower);
        robot.motorBackRight.setPower(brPower);
        robot.motorFrontRight.setPower(frPower);
        robot.motorBackLeft.setPower(blPower);





//      TELEMETRY
//        telemetry.addData("Left Motor Power", gamepad1.left_stick_y);
//        telemetry.addData("Right Motor Power", gamepad1.left_stick_y);
//        telemetry.addData("Front Motor Power", gamepad1.left_stick_x);
//        telemetry.addData("Back Motor Power", gamepad1.left_stick_x);
//        telemetry.addData("Hook Motor Power", gamepad2.left_stick_y);
//        telemetry.addData("Arm Tilt Motor Power", aPowerTIlt);
//        telemetry.addData("Arm Ext Motor Power", aPowerExt);
//        telemetry.addData("Box intake Power", boxPower);
//        telemetry.addData("Right Distance", robot.sensorRangeR.getDistance(DistanceUnit.INCH));
//        telemetry.addData("Left Distance", robot.sensorRangeL.getDistance(DistanceUnit.INCH));

    }

//--------------------------------- FUNCTIONS ----------------------------------------------------

    public double scalePower(double power1)
    {
        double power2;
        if(power1 > 0){
            power2 = Math.pow(Math.abs(power1), 0.6);
        } else if(power1 < 0){
            power2 = -Math.pow(Math.abs(power1), 0.6);
        } else{
            power2 = 0;
        }

        return power2;
    }
}