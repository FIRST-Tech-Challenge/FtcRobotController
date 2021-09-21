package teamcode.common;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.openftc.revextensions2.ExpansionHubMotor;

import java.util.concurrent.AbstractExecutorService;

public class WestCoastDriveTrain {

    private ExpansionHubMotor fl, fr, bl, br;
    Localizer localizer;

    private final double P_LINEAR = 0;
    private final double D_LINEAR = 0;
    private final double P_ROTATIONAL = 0;
    private final double D_ROTATIONAL = 0;

    private double previousVelocity;
    private double previousOmega;
    private double previousError;
    private double previousOmegaError;

    private final double WHEEL_RADIUS = 0;


    public WestCoastDriveTrain(Localizer localizer, HardwareMap hardwareMap){

    }

    public WestCoastDriveTrain(HardwareMap hardwareMap){
        fl = hardwareMap.get(ExpansionHubMotor.class,"FrontLeftDrive");
        fr = hardwareMap.get(ExpansionHubMotor.class, "FrontRightDrive");
        bl = hardwareMap.get(ExpansionHubMotor.class, "BackLeftDrive");
        br = hardwareMap.get(ExpansionHubMotor.class, "BackRightDrive");
        correctMotors();

    }

    public WestCoastDriveTrain(HardwareMap hardwareMap, Localizer localizer){
        fl = hardwareMap.get(ExpansionHubMotor.class,"FrontLeftDrive");
        fr = hardwareMap.get(ExpansionHubMotor.class, "FrontRightDrive");
        bl = hardwareMap.get(ExpansionHubMotor.class, "BackLeftDrive");
        br = hardwareMap.get(ExpansionHubMotor.class, "BackRightDrive");
        this.localizer = localizer;
        previousVelocity = 0;
        previousOmega = 0;
        correctMotors();

    }

    //TODO implement correctly
    private void correctMotors() {


    }

    public void moveToPosition(Vector2D desiredPosition, double desiredVelocity, double desiredOmega){


        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();

        Vector2D desiredPositionPointer = new Vector2D(desiredPosition.getX() - currentState.getPosition().getX() , desiredPosition.getY() - currentState.getPosition().getY());
        Vector2D newDesiredPosition = desiredPosition.add(new Vector2D(5.0 * Math.cos(desiredPositionPointer.getDirection()), 5.0 * Math.sin(desiredPositionPointer.getDirection())));

        previousError =  0;
        double steadyStateError = 0;
        previousOmegaError = 0;
        moveToRotation(currentState.getRotation() + newDesiredPosition.getDirection(), desiredOmega);

        while((Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()) > 5.0 && AbstractOpMode.currentOpMode().opModeIsActive())){

            currentState = localizer.getCurrentState();
            Vector2D positionError = desiredPosition.subtract(currentState.getPosition());
            double errorAngle = positionError.getDirection();
            //angleOfTravel += 0; // (Math.PI / 4.0)mecanum need this because all the math is shifted by pi/4
            Vector2D idealVelocity = Vector2D.fromAngleMagnitude(errorAngle, desiredVelocity);

            double recordedVelocity = currentState.getVelocity().magnitude();
            //recordedVelocity.rotate(-Math.PI / 4.0);


            double error = idealVelocity.magnitude() - recordedVelocity;
            //Vector2D crossTrackError = new Vector2D(xError, yError);
            steadyStateError += error;
            double deltaError = error - previousError;
            error *= P_LINEAR;
            deltaError *= D_LINEAR;
            error += deltaError;



//            double sign = 1.0;
//            if(error.getX() < 0 || error.getY() < 0){
//                sign = -1.0;
//            }
            //error.add(previousVelocity);
            double passedValue = error + previousVelocity;

            if(passedValue > 1.0){
                passedValue = 1.0;
            }else if(passedValue < -1.0){
                passedValue = -1.0;
            }


            previousVelocity = straightMovement(passedValue);

            // previousVelocity.multiply(sign);
            previousError = error;
            //AbstractOpMode.currentOpMode().telemetry.addData("", currentState.toString());
//
            //AbstractOpMode.currentOpMode().telemetry.addData("distance", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));
            //AbstractOpMode.currentOpMode().telemetry.addData("sign", Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude()));

            AbstractOpMode.currentOpMode().telemetry.addData("", currentState);
            //AbstractOpMode.currentOpMode().telemetry.addData("error", (Math.abs(newDesiredPosition.subtract(currentState.getPosition()).magnitude())));

            AbstractOpMode.currentOpMode().telemetry.update();
        }
        brake();
    }

    private void brake() {
        fl.setPower(0);
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        previousVelocity = 0;
    }


    public double straightMovement(double power){

        setPower(power, power, power, power);
        return power;
    }


    private int getSign(double num){
        if(num < 0){
            return -1;
        }else {
            return 1;
        }
    }


    public void moveToRotation(double desiredRotation, double desiredOmega) {
        RobotPositionStateUpdater.RobotPositionState currentState = localizer.getCurrentState();
        double currentRotation = currentState.getRotation();
        double newDesiredRotation = desiredRotation;
        if(desiredOmega > 0){
            newDesiredRotation += 0.05;
        }else{
            newDesiredRotation -= 0.05;
        }
        while(Math.abs(newDesiredRotation - currentRotation) > 0.05 && AbstractOpMode.currentOpMode().opModeIsActive()){
            currentState = localizer.getCurrentState();
            double recordedOmega = currentState.getAngularVelocity();
            double omegaError = desiredOmega - recordedOmega;
            double deltaOmegaError = omegaError - previousOmegaError;
            omegaError *= P_ROTATIONAL;
            deltaOmegaError *= D_ROTATIONAL;

            double passedOmega = previousOmega + omegaError + deltaOmegaError;

            if(passedOmega > 1.0){
                passedOmega = 1.0;
            }else if(passedOmega < -1.0){
                passedOmega = -1.0;
            }

            previousOmega = rotate(passedOmega);
        }
    }

    public double rotate(double omega) {
        setPower(omega, omega, -omega, -omega);
        return omega;
    }

    public void setPower(double flPower, double frPower, double blPower, double brPower){
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);

    }

    public void setVelocity(double flVelocity, double frVelocity, double blVelocity, double brVelocity){
        fl.setVelocity(flVelocity);
        fr.setVelocity(frVelocity);
        bl.setVelocity(blVelocity);
        br.setVelocity(brVelocity);
    }


}
