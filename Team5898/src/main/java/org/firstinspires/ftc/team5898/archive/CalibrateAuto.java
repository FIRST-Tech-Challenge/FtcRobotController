package org.firstinspires.ftc.team5898.archive;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
This op mode should be used to calibrate the bias value used in the autonomous base code.
 Robot should travel 20 inches forward. Adjust the bias value (line 26) until the robot is consistent.
 Then copy the bias value to the autonomous base code.
 */
@Disabled
@Autonomous(name="Calibrate Auto", group="Starter Code")
public class CalibrateAuto extends LinearOpMode{
    //
    DcMotor frontleft, frontright, backleft, backright;
    //Calculate encoder conversion
    int cpr = 28; //counts per rotation
    double gearRatio = 19.2;
    double diameter = 3.779;
    double cpi = (cpr * gearRatio) / (Math.PI * diameter); //counts per inch -> counts per rotation / circumference

    //
    double bias = 1.0;//adjust until your robot goes 25 inches
    //


    double conversion = cpi * bias;

    @Override
    public void runOpMode() {
        frontleft = hardwareMap.dcMotor.get("FL");
        frontright = hardwareMap.dcMotor.get("FR");
        backleft = hardwareMap.dcMotor.get("BL");
        backright = hardwareMap.dcMotor.get("BR");
        frontleft.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        backleft.setDirection(DcMotorSimple.Direction.REVERSE);//If your robot goes backward, switch this from right to left
        waitForStart();

        //
        moveToPosition(25, .2);//Don't change this line, unless you want to calibrate with different speeds
        //
    }
    //
    /*
    This function's purpose is simply to drive forward or backward.
    To drive backward, simply make the inches input negative.
     */
    public void moveToPosition(double inches, double speed) {
        //
        if (inches < 5) {
            int move = (int) (Math.round(inches * conversion));
            //
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
            frontright.setTargetPosition(frontright.getCurrentPosition() + move);
            backleft.setTargetPosition(backleft.getCurrentPosition() + move);
            backright.setTargetPosition(backright.getCurrentPosition() + move);
            //
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);
            //
            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        } else {
            int move1 = (int) (Math.round((inches - 5) * conversion));
            int movefl2 = frontleft.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movefr2 = frontright.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebl2 = backleft.getCurrentPosition() + (int) (Math.round(inches * conversion));
            int movebr2 = backright.getCurrentPosition() + (int) (Math.round(inches * conversion));
            //
            frontleft.setTargetPosition(frontleft.getCurrentPosition() + move1);
            frontright.setTargetPosition(frontright.getCurrentPosition() + move1);
            backleft.setTargetPosition(backleft.getCurrentPosition() + move1);
            backright.setTargetPosition(backright.getCurrentPosition() + move1);
            //
            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //
            frontleft.setPower(speed);
            frontright.setPower(speed);
            backleft.setPower(speed);
            backright.setPower(speed);
            //
            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            }
            //
            frontleft.setTargetPosition(movefl2);
            frontright.setTargetPosition(movefr2);
            backleft.setTargetPosition(movebl2);
            backright.setTargetPosition(movebr2);
            //
            frontleft.setPower(.1);
            frontright.setPower(.1);
            backleft.setPower(.1);
            backright.setPower(.1);
            //
            while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
            }
            frontleft.setPower(0);
            frontright.setPower(0);
            backleft.setPower(0);
            backright.setPower(0);
        }
        return;
    }
}
