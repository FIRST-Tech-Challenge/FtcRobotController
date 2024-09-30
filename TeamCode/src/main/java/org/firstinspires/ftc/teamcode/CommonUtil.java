package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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

import com.qualcomm.robotcore.hardware.ColorSensor;

import java.util.Timer;


public class CommonUtil extends LinearOpMode {

    Orientation myRobotOrientation;

    double ENC2DIST = 4593.0/102.0; //2000.0/48.0; // FW/BW
    double ENC2DIST_SIDEWAYS = 2911.0/57.0;
    ElapsedTime timer = new ElapsedTime();

    //imu init
    BHI260IMU imu;
    BHI260IMU.Parameters myIMUParameters;
    YawPitchRollAngles robotOrientation;

    //motor / servo init
    DcMotor bl = null;
    DcMotor fl = null;
    DcMotor fr = null;
    DcMotor br = null;
    DcMotor m0 = null;
    DcMotor m1 = null;
    DcMotor m2 = null;
    DcMotor m3 = null;
    Servo s1 = null;
    Servo s2 = null;
    Servo s3 = null;



    //All Our functions!

    // Initialize
    public void initialize(HardwareMap hardwareMap){

        //setup
        telemetry.setAutoClear(true);

        // map imu
        imu = hardwareMap.get(BHI260IMU.class,"imu");
        myIMUParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,RevHubOrientationOnRobot.UsbFacingDirection.UP )
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
        m0 = hardwareMap.get(DcMotor.class, "M0");
        m1 = hardwareMap.get(DcMotor.class, "M1");
        m2 = hardwareMap.get(DcMotor.class, "M2");
        m3 = hardwareMap.get(DcMotor.class, "M3");
        s1 = hardwareMap.get(Servo.class, "s1");
        s2 = hardwareMap.get(Servo.class, "s2");
        s3 = hardwareMap.get(Servo.class, "s3");
        s3 = hardwareMap.get(Servo.class, "s3");
        s1.setDirection(Servo.Direction.FORWARD);
        s2.setDirection(Servo.Direction.FORWARD);
        s3.setDirection(Servo.Direction.REVERSE);
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

    public void drone_Test(){
        s3.setPosition(0);
    }

    //motor power 0
    public void setMotorToZeroPower()
    {
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);
    }

    public void setZeroPowerBehavior(){

        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public double PID_Turn (double targetAngle, double currentAngle, String minPower) {
        double sign = 1;
        double power = (targetAngle - currentAngle) * 0.01; // was 0.05
        if (minPower.equalsIgnoreCase("on")&& (power != 0)) {
            sign = Math.signum(power);
            power = Math.max(Math.abs(power), 0.1);
            power = power*sign;
        }
        return power;
    }

    public double PID_FB (double targetEC, double currentEC)
    {
        double power = (targetEC -currentEC)*0.0003;
        if (power < 0)
        {
            power = 0;
        }
        else if (power < 0.1)
        {
            power = 0.1;
        }
        return power;
    }

    public int amIStuck_FB(double encoderAbsCounts, double currEncoderCount, double prevEncoderCount)
    {
        double currErrEC = 0;
        if (timer.time()> 0.5) // wait for passing of 0.5 second
        {
            currErrEC = Math.abs(encoderAbsCounts - Math.abs(currEncoderCount));
            if ((currErrEC/encoderAbsCounts)<0.98) {
                if (Math.abs(currEncoderCount) - Math.abs(prevEncoderCount) <100)
                {
                    telemetry.addData("fw:stuck","yes");
                    telemetry.update();
                    sleep(100000);
                    timer.reset();
                    return(1);
                }
                else {
                    timer.reset();
                    return(0);
                }
            }
        }
        return(-1);
    }

    //move forwards with gyro
    public int moveForward_wDistance_wGyro(double DistanceAbsIn,double Mpower)
    {
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
        while (bl.getCurrentPosition() < encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            currZAngle = myRobotOrientation.thirdAngle;
            double correction = PID_Turn(0,currZAngle,"off");
            currEncoderCount = bl.getCurrentPosition();
            double power = PID_FB(encoderAbsCounts,Math.abs(currEncoderCount));

            bl.setPower(power-correction);
            fl.setPower(power-correction);
            fr.setPower(power+correction);
            br.setPower(power+correction);
            telemetry.addData("fw:power", power);
            telemetry.addData("fw:correction", correction);
            telemetry.update();

            // quick correct for angle if it is greater than 10 [Aarush]
            double absError_angle = Math.abs(currZAngle);
            if (absError_angle > 10)
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
        turnToZeroAngle();

        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currZAngle = myRobotOrientation.thirdAngle;
        telemetry.addData("fw:currEncoderCount", currEncoderCount);
        telemetry.addData("fw:currZAngle", currZAngle);
        telemetry.update();
        return (currEncoderCount);
    }

    //move backwards with gyro correction
    public int moveBackwards_wDistance_wGyro(double DistanceAbsIn,double Mpower)
    {
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
        while(bl.getCurrentPosition() > -encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            currZAngle = myRobotOrientation.thirdAngle;
            double correction = PID_Turn(0,currZAngle,"off");
            currEncoderCount = bl.getCurrentPosition();
            double power = PID_FB(encoderAbsCounts,Math.abs(currEncoderCount));

            bl.setPower(-power-correction);
            fl.setPower(-power-correction);
            fr.setPower(-power+correction);
            br.setPower(-power+correction);
            telemetry.addData("bw:power", power);
            telemetry.addData("bw:correction", correction);
            telemetry.update();

            // quick correct for angle if it is greater than 10 [Aarush]
            double absError_angle = Math.abs(currZAngle);
            if (absError_angle > 10)
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

            idle();
        }

        turnToZeroAngle();
        // apply zero power to avoid continuous power to the wheels
        setMotorToZeroPower();

        // return current encoder count
        currEncoderCount = bl.getCurrentPosition();
        myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
        currZAngle = myRobotOrientation.thirdAngle;
        telemetry.addData("bw:currEncoderCount", currEncoderCount);
        telemetry.addData("bw:currZAngle", currZAngle);
        telemetry.update();
        return (currEncoderCount);
    }

    public void clawClosed()
    {
        s1.setPosition(0.2);
    }


    public void wristFlat()
    {
        s2.setPosition(0.147);
    }

    public void clawOpen()
    {
        s1.setPosition(0.4);
    }

    public void wristBent()
    {
        s2.setPosition(0.427);
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
    public void intake(int t_msec )
    {
        m0.setDirection(DcMotor.Direction.FORWARD);
        m1.setDirection(DcMotor.Direction.REVERSE);
        m0.setPower(1);
        m1.setPower(1);
        sleep(t_msec);
        m0.setPower(0);
        m1.setPower(0);
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

    public void encoder_test(double encoderAbsCounts)
    {
        bl.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.REVERSE);
        fr.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.FORWARD);
        resetMotorEncoderCounts();
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (bl.getCurrentPosition() > -encoderAbsCounts) {
            myRobotOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            bl.setPower(-0.3);
            fl.setPower(-0.3);
            fr.setPower(-0.3);
            br.setPower(-0.3);
            telemetry.update();
            idle();
        }
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);


        telemetry.addData("encoder count b1",bl.getCurrentPosition());
        telemetry.addData("encoder count br",br.getCurrentPosition());
        telemetry.addData("encoder count fl",fl.getCurrentPosition());
        telemetry.addData("encoder count fr",fr.getCurrentPosition());
        telemetry.update();

    }

    public void extend(double power, int encoderAbsCounts) {
        m2.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.FORWARD);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Start count", m2.getCurrentPosition());
        telemetry.addData("Start count", m3.getCurrentPosition());
        telemetry.update();

        while (m2.getCurrentPosition() > -encoderAbsCounts){
            m2.setPower(-power);
            m3.setPower(power);
            telemetry.addData("Count M2",m2.getCurrentPosition());
            telemetry.addData("Count M3",m3.getCurrentPosition());
            telemetry.update();
            idle();
        }
        m2.setPower(0); // set power to 0 so the motor stops running
        m3.setPower(0);

    }

    public void retract(double power, int encoderAbsCounts) {
        m2.setDirection(DcMotor.Direction.FORWARD);
        m3.setDirection(DcMotor.Direction.FORWARD);
        m2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        m2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        m3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("Start count", m2.getCurrentPosition());
        telemetry.addData("Start count", m3.getCurrentPosition());
        telemetry.update();

        while (m2.getCurrentPosition() < encoderAbsCounts){
            m2.setPower(power);
            m3.setPower(-power);
            telemetry.addData("Count M2",m2.getCurrentPosition());
            telemetry.addData("Count M3",m3.getCurrentPosition());
            telemetry.update();
            idle();
        }
        m2.setPower(0); // set power to 0 so the motor stops running
        m3.setPower(0);

    }

    public void realign_Sideways(String direction){


        if (direction.equalsIgnoreCase("left")) {
            bl.setPower(-0.2);
            fl.setPower(0.2);
            fr.setPower(0.2);
            br.setPower(-0.2);
        }
        else if (direction.equalsIgnoreCase("right")) {
            bl.setPower(0.2);
            fl.setPower(-0.2);
            fr.setPower(-0.2);
            br.setPower(0.2);
        }
        sleep(1500);
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }

    public void realign_FB(String direction){
        if (direction.equalsIgnoreCase("forward")) {
            bl.setPower(0.2);
            fl.setPower(0.2);
            fr.setPower(0.2);
            br.setPower(0.2);
        }
        else if (direction.equalsIgnoreCase("backward")) {
            bl.setPower(-0.2);
            fl.setPower(-0.2);
            fr.setPower(-0.2);
            br.setPower(-0.2);
        }
        sleep(1500);
        bl.setPower(0);
        fl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

    }


    @Override
    public void runOpMode() throws InterruptedException {
    }


}
