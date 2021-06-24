package teamcode.common;

import android.sax.StartElementListener;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;

public class MecanumDriveTrain {
    private static final double ANGULAR_TOLERANCE = 0.05;
    final double COUNTS_PER_INCH = 920.111004;

    /*
    This has most of the relevant information regarding a 4 wheel Mechanum DriveTrain,
    which is the most used DriveTrain in FTC
     */

    private DcMotor fl, fr, bl, br;
    Localizer localizer;
    Vector2D previousVelocity;
    Vector2D previousError;

    /**
     * PID Constants
     *
     */
    final double pVelocity = 0.001; //0.001
    final double dVelocity  = 0.05;
    final double pOmega = 0.225;

    public MecanumDriveTrain(HardwareMap hardwareMap){
        fl = hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = hardwareMap.dcMotor.get("FrontRightDrive");
        bl = hardwareMap.dcMotor.get("BackLeftDrive");
        br = hardwareMap.dcMotor.get("BackRightDrive");
        correctMotors();

    }

    public MecanumDriveTrain(HardwareMap hardwareMap, Localizer localizer){
        fl = hardwareMap.dcMotor.get("FrontLeftDrive");
        fr = hardwareMap.dcMotor.get("FrontRightDrive");
        bl = hardwareMap.dcMotor.get("BackLeftDrive");
        br = hardwareMap.dcMotor.get("BackRightDrive");
        this.localizer = localizer;
        previousVelocity = new Vector2D(0,0);
        correctMotors();

    }


    private void correctMotors() {
        fl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }






    /**
     * moving from position to position
     * @param desiredPosition the end point of the robot
     * @param desiredVelocity the end velocity of the robot in inches per second
     * @param desiredAngle the angle the robot should be at at the end.
     */

    public void moveToPosition(Vector2D desiredPosition, double desiredVelocity, double desiredAngle){


        Vector2D newDesiredPosition = desiredPosition.add(new Vector2D(5.0 * Math.cos(desiredPosition.getDirection()), 5.0 * Math.sin(desiredPosition.getDirection())));
        previousError = new Vector2D(0,0);

        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();
        while((Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()) > 5.0 || Math.abs(currentState.getRotation() - desiredAngle) > 0.05) && AbstractOpMode.currentOpMode().opModeIsActive()){

            currentState = localizer.getCurrentState();
            Vector2D positionError = desiredPosition.subtract(currentState.getPosition());
            double errorAngle = positionError.getDirection();
            //angleOfTravel += 0; // (Math.PI / 4.0)mecanum need this because all the math is shifted by pi/4
            Vector2D idealVelocity = Vector2D.fromAngleMagnitude(errorAngle, desiredVelocity);

            Vector2D recordedVelocity = currentState.getVelocity();
            //recordedVelocity.rotate(-Math.PI / 4.0);

            double currentRotation = currentState.getRotation();
            double rotationError =  (desiredAngle - currentRotation);


            double recordedOmega = currentState.getAngularVelocity();
            double desiredOmega = Math.PI;



            double xError = (idealVelocity.getX() - recordedVelocity.getX());
            double yError = (idealVelocity.getY() - recordedVelocity.getY());
            double omegaError =  (desiredOmega - recordedOmega);
            Vector2D error = new Vector2D(xError, yError);
            //Vector2D deltaError = error.subtract(previousError);
            error = error.multiply(pVelocity);
           // deltaError = deltaError.multiply(dVelocity);
            //error.add(deltaError);
            omegaError *= pOmega;
            omegaError *= rotationError;

//            double sign = 1.0;
//            if(error.getX() < 0 || error.getY() < 0){
//                sign = -1.0;
//            }

            previousVelocity = setPowerPurePursuit(error.add(previousVelocity), omegaError);
           // previousVelocity.multiply(sign);
            previousError = error;

            //AbstractOpMode.currentOpMode().telemetry.addData("", currentState.toString());
//
            AbstractOpMode.currentOpMode().telemetry.addData("distance", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));
            AbstractOpMode.currentOpMode().telemetry.addData("sign", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));


            AbstractOpMode.currentOpMode().telemetry.addData("", currentState);
            AbstractOpMode.currentOpMode().telemetry.update();
        }
        brake();


    }


    double previousOmega;
    double pRotation;
    public void moveToRotation(double desiredRotation, double omega){
        RobotPositionStateUpdater.RobotPositionState state = localizer.getCurrentState();
        previousOmega = 0;
        while(Math.abs(desiredRotation - state.getRotation()) > 0.02){
            state = localizer.getCurrentState();
            double recordedOmega = state.getAngularVelocity();
            double omegaError = omega - recordedOmega;
            omegaError *= pRotation;
            omega += omegaError;
            setPowerPurePursuit(new Vector2D(0,0), omega);
            AbstractOpMode.currentOpMode().telemetry.addData("", state.toString());
            AbstractOpMode.currentOpMode().telemetry.update();
        }
        brake();
    }

    private void brake() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        previousVelocity = new Vector2D(0,0);
    }

    public DcMotor[] getMotors(){
        return new DcMotor[]{fl,fr,bl,br};
    }


    /*
    gets the robot driving in a specified direction
     */
    public void setPower(Vector2D velocity, double turnValue){
        turnValue = -turnValue;
        double direction = velocity.getDirection();


        double power = velocity.magnitude();

        double angle = direction + 3*Math.PI / 4.0;
        double sin = Math.sin(angle);
        double cos = Math.cos(angle);

        setPower((power * sin - turnValue),(power * cos + turnValue),
                (power * cos - turnValue), (power * sin + turnValue));
    }

    /*
    this exists because of the differences between the FTC controller and raw vectors
     */
    public Vector2D setPowerPurePursuit(Vector2D velocity, double turnValue){

        turnValue = turnValue;
        double direction = velocity.getDirection();
        double power = velocity.magnitude();

        double angle = direction +  Math.PI / 4;

        double sin = Math.sin(angle);
        double cos = Math.cos(angle);
        // right movement
        // fl +, fr -, bl -, br +
        // positive clockwise
        // fl +, fr -, bl +, br -
        setPowerCorrect(power * sin + turnValue, -power * cos - turnValue,
                -power * cos + turnValue, power * sin - turnValue);
        return velocity;
    }

    public void setPowerCorrect(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(-blPow);
        br.setPower(brPow);
    }

    public void setPower(double flPow, double frPow, double blPow, double brPow) {
        fl.setPower(flPow);
        fr.setPower(frPow);
        bl.setPower(blPow);
        br.setPower(-brPow);
    }

    private boolean isNear(double globalRads, double angle, boolean isBig) {
        if (isBig) {
            return Math.abs(globalRads - angle) < (2 * ANGULAR_TOLERANCE);
        }else {
            return Math.abs(globalRads - angle) < (ANGULAR_TOLERANCE);
        }
    }

    public void zero() {
        setPower(0,0,0,0);
    }

    private int getSign(double num){
        if(num < 0){
            return -1;
        }else {
            return 1;
        }
    }







}
