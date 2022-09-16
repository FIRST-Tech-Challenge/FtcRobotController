package org.firstinspires.ftc.teamcode.League1.Subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.League1.Common.OpModeWrapper;
import org.firstinspires.ftc.teamcode.League1.Common.Point;
import org.firstinspires.ftc.teamcode.League1.Common.Robot;
import org.firstinspires.ftc.teamcode.League1.Common.Utils;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintStream;

public class MecDrive {

    private DcMotorEx fl, fr, bl, br;
    private Robot robot;
    private boolean isDriveOnChub = true;
    private boolean pidEnabled;
    Telemetry telemetry;


    File loggingFile = AppUtil.getInstance().getSettingsFile("telemetry.txt");
    String loggingString;

    /*
    localizer -> Arm/actuator -> drive
     */

    public MecDrive(HardwareMap hardwareMap, Robot robot , boolean pidEnabled, Telemetry telemetry){
        fl = hardwareMap.get(DcMotorEx.class, "FrontLeftDrive");
        fr = hardwareMap.get(DcMotorEx.class, "FrontRightDrive");
        bl = hardwareMap.get(DcMotorEx.class, "BackLeftDrive");
        br = hardwareMap.get(DcMotorEx.class, "BackRightDrive");
        this.robot = robot;
        this.pidEnabled = pidEnabled;
        this.telemetry = telemetry;

        robot.setShouldUpdate(false);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setShouldUpdate(true);







        CorrectMotors();
    }

    public void rotate(double angle, double power){
        if(angle < 0){
            power *= -1;
        }

        angle = Utils.wrapAngle(angle);
        if(pidEnabled){
            setPIDRotateVelocity(angle);

        }else {

            while (Math.abs(robot.getDirection()) < Math.abs(angle)) {


                setPower(-power, power, -power, power);

            }
        }

        brake();

    }




    public void newMoveToPosition(Point p, double power) {
        double direction = Utils.wrapAngle(p.getDirection()) - (Math.PI / 4);

        int flPosition = Math.abs((int) (Math.cos(direction) * p.magFromOrigin()));
        int frPosition = Math.abs((int) (Math.sin(direction) * p.magFromOrigin()));
        int blPosition = Math.abs((int) (Math.sin(direction) * p.magFromOrigin()));
        int brPosition = Math.abs((int) (Math.cos(direction) * p.magFromOrigin()));

        /*double flVelocity = Math.max(1, velocity * Math.cos(direction));
        double frVelocity = Math.max(1, velocity * Math.sin(direction));
        double blVelocity = Math.max(1, velocity * Math.sin(direction));
        double brVelocity = Math.max(1, velocity * Math.cos(direction));
*/
        double flPower = Math.cos(direction) * power;
        double frPower = Math.sin(direction) * power;
        double blPower = Math.sin(direction) * power;
        double brPower = Math.cos(direction) * power;


        if (pidEnabled) {
            //setPIDVelocity(flVelocity, flPosition, frVelocity, frPosition, blVelocity, blPosition, brVelocity, brPosition);
        } else {
            LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);

            int flPos = data.getMotorCurrentPosition(0);
            int frPos = data.getMotorCurrentPosition(1);
            int blPos = data.getMotorCurrentPosition(2);
            int brPos = data.getMotorCurrentPosition(3);

            //TODO: might need to change this direction thing
            if(direction + (Math.PI/4) > 0) {
                frPos *= -1;
                brPos *= -1;
            }else{
                flPos *= -1;
                blPos *= -1;
            }
            //TODO:check if might need to change abs condition
            while (opModeIsRunning() && (flPos < flPosition || frPos < frPosition || blPos < blPosition || brPos < brPosition)) {
                //setMotorVelocity(flVelocity, frVelocity, blVelocity, brVelocity);

                if (flPos >= flPosition) {
                    flPower = 0;

                } else if (frPos >= frPosition) {
                    frPower = 0;

                } else if (blPos >= blPosition) {
                    blPower = 0;

                } else if (brPos >= brPosition) {
                    brPower = 0;

                }

                setPower(flPower, frPower, blPower, brPower);


                //setPower(0.3, 0.3, 0.3, 0.3);
                data = robot.getBulkPacket(isDriveOnChub);

                flPos = data.getMotorCurrentPosition(0);
                frPos = data.getMotorCurrentPosition(1);
                blPos = data.getMotorCurrentPosition(2);
                brPos = data.getMotorCurrentPosition(3);

                if(direction + (Math.PI/4) > 0) {
                    frPos *= -1;
                    brPos *= -1;
                }else{
                    flPos *= -1;
                    blPos *= -1;
                }

                //Telemetry and logging for current motor positions
                telemetry.addData("fl: ", flPos);
                telemetry.addData("fr: ", frPos);
                telemetry.addData("bl: ", blPos);
                telemetry.addData("br: ", brPos);

                loggingString += "flCurrentPosition: " + flPos + "\n";
                loggingString += "frCurrentPosition: " + flPos + "\n";
                loggingString += "blCurrentPosition: " + flPos + "\n";
                loggingString += "brCurrentPosition: " + flPos + "\n";
                loggingString += "\n";

                //Telemetry and logging for target motor positions
                telemetry.addData("target fl: ", flPosition);
                telemetry.addData("target fr: ", frPosition);
                telemetry.addData("target bl: ", blPosition);
                telemetry.addData("target br: ", brPosition);

                loggingString += "flTargetPosition: " + flPosition + "\n";
                loggingString += "frTargetPosition: " + frPosition + "\n";
                loggingString += "blTargetPosition: " + blPosition + "\n";
                loggingString += "brTargetPosition: " + brPosition + "\n";
                loggingString += "\n";

                //Telemetry and logging for power that each motor is set to
                telemetry.addData("flPower: ", flPower);
                telemetry.addData("frPower: ", frPower);
                telemetry.addData("blPower: ", blPower);
                telemetry.addData("brPower: ", brPower);

                loggingString += "flPower: " + flPower + "\n";
                loggingString += "frPower: " + frPower + "\n";
                loggingString += "blPower: " + blPower + "\n";
                loggingString += "brPower: " + brPower + "\n";
                loggingString += "\n";
                loggingString += "\n";
                loggingString += "\n";



                telemetry.update();

            }
            brake();
        }
    }




        public void moveToPosition(Point p, double velocity){

        double robotDirection = robot.getDirection();
        double direction = Utils.wrapAngle(p.getDirection()) - Utils.wrapAngle(robotDirection);

        int flPosition = (int)(Math.sin(direction) * p.magFromOrigin()); //Direction Math.Pi/4 -> 0
        int frPosition = (int)(Math.cos(direction) * p.magFromOrigin());
        int blPosition = (int)(Math.cos(direction) * p.magFromOrigin());
        int brPosition = (int)(Math.sin(direction) * p.magFromOrigin());

        double flVelocity = Math.max(1, velocity * Math.sin(direction));
        double frVelocity = Math.max(1, velocity * Math.cos(direction));
        double blVelocity = Math.max(1, velocity * Math.cos(direction));
        double brVelocity = Math.max(1, velocity * Math.sin(direction));


/*

        double flPower = Math.sin(direction) * 0.5;
        double frPower = Math.cos(direction) * 0.5;
        double blPower = Math.cos(direction) * 0.5;
        double brPower = Math.sin(direction) * 0.5;
*/



        if(pidEnabled){
            setPIDVelocity(flVelocity, flPosition, frVelocity, frPosition, blVelocity, blPosition, brVelocity, brPosition);
        }else {
            LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);

            int flPos = data.getMotorCurrentPosition(0);
            int frPos = data.getMotorCurrentPosition(1);
            int blPos = data.getMotorCurrentPosition(2);
            int brPos = data.getMotorCurrentPosition(3);
            while (opModeIsRunning() && flPos < flPosition && frPos < frPosition && blPos < blPosition && brPos < brPosition) {
                //setMotorVelocity(flVelocity, frVelocity, blVelocity, brVelocity);

                setPower(0.3, 0.3, 0.3, 0.3);
                data = robot.getBulkPacket(isDriveOnChub);

                flPos = data.getMotorCurrentPosition(0);
                frPos = data.getMotorCurrentPosition(1);
                blPos = data.getMotorCurrentPosition(2);
                brPos = data.getMotorCurrentPosition(3);

                telemetry.addData("fl: ", flPos);
                telemetry.addData("fr: ", frPos);
                telemetry.addData("bl: ", blPos);
                telemetry.addData("br: ", brPos);

                telemetry.addData("target fl: ", flPosition);
                telemetry.addData("target fr: ", frPosition);
                telemetry.addData("target bl: ", blPosition);
                telemetry.addData("target br: ", brPosition);


                telemetry.update();

            }
            brake();
        }
    }

    private void brake() {
        robot.setShouldUpdate(false);
        setPower(0,0,0,0);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setShouldUpdate(true);

    }

    private void setMotorVelocity(double flV, double frV, double blV, double brV){
        fl.setVelocity(flV);
        fr.setVelocity(frV);
        bl.setVelocity(blV);
        br.setVelocity(brV);
    }

    private void setMotorPower(double flP, double frP, double blP, double brP){
        fl.setPower(flP);
        fr.setPower(frP);
        bl.setPower(blP);
        br.setPower(brP);
    }


    private double P = 0.1; //TODO starting values to be tuned with Zeiger Nichols method
    private double I = 0;
    private double D = 0;
    private double F = 0;

    double integralSumLimit = 0.3;

    /**
     *
     * @param flV target velocity d/dx position
     * @param flPosition target
     * Zeiger-Nichols method
     * low pass filtering
     */
    private void setPIDVelocity(double flV, int flPosition, double frV, int frPosition, double blV, int blPosition, double brV, int brPosition){
        double flIntegralSum = 0;
        double flPreviousError = 0;
        double frIntegralSum = 0;
        double frPreviousError = 0;
        double blIntegralSum = 0;
        double blPreviousError = 0;
        double brIntegralSum = 0;
        double brPreviousError = 0;

        double start = OpModeWrapper.currentOpMode().time; //start time
        LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);
        int flCurr = data.getMotorCurrentPosition(0); //init motor pos'
        int frCurr = data.getMotorCurrentPosition(1);
        int blCurr = data.getMotorCurrentPosition(2);
        int brCurr = data.getMotorCurrentPosition(3);

        //low pass fields
//        double tau = 0.8;
//        double previousFilterEstimate = 0;
//        double currentFilterEstimate = 0;

        while(opModeIsRunning() && flCurr < flPosition && frCurr < frPosition && blCurr < blPosition && brCurr < brPosition){

            data = robot.getBulkPacket(isDriveOnChub);
            flCurr = data.getMotorCurrentPosition(0);
            frCurr = data.getMotorCurrentPosition(1);
            blCurr = data.getMotorCurrentPosition(2);
            brCurr = data.getMotorCurrentPosition(3);


            double flError = flPosition - flCurr; //error, P
            double frError = frPosition - frCurr;
            double blError = blPosition - blCurr;
            double brError = brPosition - brCurr;

            //low pass impl, note to write this for ALL motors if implemented, keep Tau constant among all 4 wheels/sensors
            //currentFilterEstimate = (tau * previousFilterEstimate) + (1-tau) * flError - previousError;
            //double flDerivative = currentFilterEstimate / elapsedTime;

            double elapsedTime = OpModeWrapper.currentOpMode().time - start; //elapsed time since start of the movement

            double flDerivative = (flError - flPreviousError) / elapsedTime; //Rate of Change, Error/time
            double frDerivative = (frError - frPreviousError) / elapsedTime;
            double blDerivative = (blError - blPreviousError) / elapsedTime;
            double brDerivative = (brError - brPreviousError) / elapsedTime;


            flIntegralSum = flIntegralSum + (flCurr * elapsedTime); //sum of error
            frIntegralSum = frIntegralSum + (frCurr * elapsedTime);
            blIntegralSum = blIntegralSum + (blCurr * elapsedTime);
            brIntegralSum = brIntegralSum + (brCurr * elapsedTime);

            if(flIntegralSum > integralSumLimit){
                flIntegralSum = integralSumLimit;
            }else if (flIntegralSum < - integralSumLimit){
                flIntegralSum = -integralSumLimit;
            }
            if(frIntegralSum > integralSumLimit){
                frIntegralSum = integralSumLimit;
            }else if (frIntegralSum < - integralSumLimit){
                frIntegralSum = -integralSumLimit;
            }
            if(blIntegralSum > integralSumLimit){
                blIntegralSum = integralSumLimit;
            }else if (blIntegralSum < - integralSumLimit){
                blIntegralSum = -integralSumLimit;
            }
            if(brIntegralSum > integralSumLimit){
                brIntegralSum = integralSumLimit;
            }else if (brIntegralSum < - integralSumLimit){
                brIntegralSum = -integralSumLimit;
            }


            double flOut = (P * flError) + (I * flIntegralSum) + (D * flDerivative) + F;
            double frOut = (P * frError) + (I * frIntegralSum) + (D * frDerivative) + F;
            double blOut = (P * blError) + (I * blIntegralSum) + (D * blDerivative) + F;
            double brOut = (P * brError) + (I * brIntegralSum) + (D * brDerivative) + F;

            setMotorVelocity(flOut * flV, frOut * frV,blOut * blV,brOut * brV);

            flPreviousError = flError;
            frPreviousError = frError;
            blPreviousError = blError;
            brPreviousError = brError;
            //previousFilterEstimate = currentFilterEstimate;
        }
        brake();
    }



    PIDCoefficients pid = new PIDCoefficients(0,0,0);

    //TODO: see if can make more efficient using timer.reset() and maybe add pid for each motor
    private void setPIDRotateVelocity(double targetAngle){
        ElapsedTime time = new ElapsedTime();
        double startTime = time.seconds();
        double integralSum = 0;

        //TODO: Check this line (check logic)
        double previousError = targetAngle;


        while(opModeIsRunning() && robot.getDirection() < targetAngle){
            double error = Utils.wrapAngle(targetAngle - robot.getDirection());
            double currentTime = time.seconds();

            //TODO: add an integral sum limit
            integralSum += (0.5 * (currentTime - startTime) * (previousError + error));


            double derivative = (error - previousError)/(currentTime - startTime);

            double power = (pid.p * error) + (pid.i * integralSum) + (pid.d * derivative);

            if(targetAngle < 0){
                power *= -1;
            }

            setPower(-power, power, -power, power);



            previousError = error;
            startTime = currentTime;

        }

        brake();


    }


    private boolean opModeIsRunning(){
        return OpModeWrapper.currentOpMode().opModeIsActive() && !OpModeWrapper.currentOpMode().isStopRequested();
    }
    /*
    46

     */

    private void CorrectMotors() {
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    /*
    public synchronized void driveAuto(double desiredVelocity, int tics, MecanumDriveTrain.MovementType movement){

        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);


        OpModeWrapper currentOpMode = OpModeWrapper.currentOpMode();
        double startTime = currentOpMode.time;

        if(tics < 0) {
            desiredVelocity *= -1;
        }
        double elapsedTime = 0;

        while(isFar(tics) && currentOpMode.opModeIsActive() && elapsedTime < 5.0){
            elapsedTime = currentOpMode.time - startTime;
            setPowerAuto(desiredVelocity, movement);
            data = robot.getBulkPacket(isDriveOnChub);
            telemetry.addData("fl ", data.getMotorCurrentPosition(0));
            telemetry.addData("fr ", data.getMotorCurrentPosition(1));
            telemetry.addData("bl ", data.getMotorCurrentPosition(2));
            telemetry.addData("br ", data.getMotorCurrentPosition(3));
            telemetry.update();

        }

        brake();
    }

     */

    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(-flPow);
        fr.setPower(frPow) ;
        bl.setPower(-blPow);
        br.setPower(brPow);
    }

/*
    public double setPowerAuto(double power, MecanumDriveTrain.MovementType movement) {
        if(movement == MecanumDriveTrain.MovementType.STRAIGHT) {
            setPower(power, power, power, power);
        }else if(movement == MecanumDriveTrain.MovementType.STRAFE){
            setPower(power, -power, -power, power);
        }else if(movement == MecanumDriveTrain.MovementType.ROTATE){
            setPower(power, -power, power, -power);
        }else if(movement == MecanumDriveTrain.MovementType.LDIAGONAL){
            setPower(power, 0, 0, power);
        }else if(movement == MecanumDriveTrain.MovementType.RDIAGONAL){
            setPower(0, power, power, 0);
        }
        return power;
    }

 */
    //random comment
    private double TIC_TOLERANCE = 25;
    private boolean isFar(int tics){
        LynxModule.BulkData data = robot.getBulkPacket(isDriveOnChub);

        return Math.abs(tics - data.getMotorCurrentPosition(0)) > TIC_TOLERANCE && Math.abs(tics - data.getMotorCurrentPosition(1)) > TIC_TOLERANCE
                && Math.abs(tics - data.getMotorCurrentPosition(2)) > TIC_TOLERANCE && Math.abs(tics - data.getMotorCurrentPosition(3)) > TIC_TOLERANCE;
    }

    public void writeLoggerToFile(){
        try{
            PrintStream toFile = new PrintStream(loggingFile);
            toFile.println(loggingString);
        }catch(FileNotFoundException e){
            e.printStackTrace();
        }
    }
}