package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class HWC {
    public DcMotorEx frontL, frontR, backL, backR, duckWheel, arm;
    public DcMotor extender;
    public CRServo intakeL, intakeR;
    public DistanceSensor dSensorR;
    Telemetry telemetry;
    boolean isDriving = false;
    boolean isTurning = false;
    boolean readingDuck = false;
    public static final double ONE_CM_IN_PPR = 7.9;
    public static final double ARM_VELOCITY = 20000;
    public static final double ONE_DEGREE_IN_PPR = 4.27;
    public static final double EXTENDER_POWER = 0.5;
    double rightTargetPosition = 0;
    double leftTargetPosition = 0;

    public HWC ( HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        
        duckWheel = hardwareMap.get(DcMotorEx.class, "duckWheel");
        frontL = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontR = hardwareMap.get(DcMotorEx.class, "rightFront");
        backL = hardwareMap.get(DcMotorEx.class, "leftRear");
        backR = hardwareMap.get(DcMotorEx.class, "rightRear");
        extender = hardwareMap.get(DcMotor.class, "extender");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        dSensorR = hardwareMap.get(DistanceSensor.class, "dSensorR");
        //clawLift = hardwareMap.get(DcMotor.class, " intakeLift");


        frontL.setDirection(DcMotorEx.Direction.FORWARD);
        backL.setDirection(DcMotorEx.Direction.FORWARD);
        frontR.setDirection(DcMotorEx.Direction.REVERSE);
        backR.setDirection(DcMotorEx.Direction.REVERSE);
        extender.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        intakeL.setDirection(CRServo.Direction.REVERSE);
        intakeR.setDirection(CRServo.Direction.FORWARD);

        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
       // clawLift.setZeroPowerBehavior(DCMotor.ZeroPowerBehavior.BRAKE);

        frontL.setMode(RUN_USING_ENCODER);
        frontR.setMode(RUN_USING_ENCODER);
        backL.setMode(RUN_USING_ENCODER);
        backR.setMode(RUN_USING_ENCODER);
        arm.setMode(RUN_USING_ENCODER);
        extender.setMode(RUN_WITHOUT_ENCODER);
        //clawLift.setMode(RUN_USING_ENCODER);
    }

    public void drive(double directionInDegrees, double distanceInCm, double wheelPower){
//      384.5(PPR) = ~50cm = ~20in
//      7.9(PPR) = 1cm
//        4.27(PPR) = 1 Degree
        double pprForward = distanceInCm * ONE_CM_IN_PPR;

    }

    public void armTopLayer() {
        arm.setTargetPosition(-9000);
        arm.setMode(RUN_TO_POSITION);
        arm.setVelocity(ARM_VELOCITY);
        while (arm.isBusy()) {
            telemetry.addData("Status", "Waiting for Motors to Finish Turning");
            telemetry.addData("Motors", "Arm Position: %d, %d", arm.getCurrentPosition(), arm.getTargetPosition());
            telemetry.update();
        }

        extender.setPower(EXTENDER_POWER);
//        sleep(1200);
        extender.setPower(0);
        intakeR.setPower(1);
        intakeL.setPower(1);
    }

    public void armReset() {
//        sleep(1200);
        intakeR.setPower(0);
        intakeL.setPower(0);
        extender.setPower(-0.6);
//        sleep(500);
        extender.setPower(-0.2);
//        sleep(500);
        extender.setPower(0);
        arm.setTargetPosition(0);
        arm.setMode(RUN_TO_POSITION);
        arm.setVelocity(HWC.ARM_VELOCITY);
        while (arm.isBusy()) {
            telemetry.addData("Status", "Waiting for Motors to Finish Turning");
            telemetry.addData("Motors", "Arm Position: %d, %d", arm.getCurrentPosition(), arm.getTargetPosition());
            telemetry.update();
        }
    }

    public void stopDriving(){
        isDriving = false;
        isTurning = false;
        frontL.setPower(0);
        frontR.setPower(0);
        backR.setPower(0);
        backL.setPower(0);
    }

    public void armMiddleLayer(){
        arm.setTargetPosition(-12000);
        arm.setMode(RUN_TO_POSITION);
        arm.setVelocity(ARM_VELOCITY);
        while (arm.isBusy()) {
            telemetry.addData("Status", "Waiting for Motors to Finish Turning");
            telemetry.addData("Motors", "Arm Position: %d, %d", arm.getCurrentPosition(), arm.getTargetPosition());
            telemetry.update();
        }
    }

    public void armBottomLayer(){
        arm.setTargetPosition(-7486);
        arm.setMode(RUN_TO_POSITION);
        arm.setVelocity(ARM_VELOCITY);
        while (arm.isBusy()) {
            telemetry.addData("Status", "Waiting for Motors to Finish Turning");
            telemetry.addData("Motors", "Arm Position: %d, %d", arm.getCurrentPosition(), arm.getTargetPosition());
            telemetry.update();
        }
    }

    public void findDuck(){

        double cmToDucky = 0.0;
        final double distanceToShippingHubPPR = 5.0;
        final double distanceBetweenBarcodes = 5; // CM | TODO: Change Later with exact value
        int duckSpot = 0;

        while(readingDuck){
            drive(2, 0, 500);
            if(dSensorR.getDistance(DistanceUnit.INCH) <= 40){ //TODO: Subtract Robot, Maybe 15?
                if(cmToDucky >= 9 && cmToDucky <= 18){ // Barcode1 means that it is the Bottom Layer and it will be within 2 cm each side
                    // You can check how far you moved by getting the value of the cmToDucky variable.

                    // TODO: Check exact distances for these two if statements
                    readingDuck = false;
                    duckSpot = 2;
                } else if(cmToDucky > 18 && cmToDucky <= 27){
                    readingDuck = false;
                    duckSpot = 3;
                }
            } else if(cmToDucky > 27) { //TODO: Actual End of all Spots Length
                duckSpot = 1;
                readingDuck = false;
            }
            cmToDucky = frontL.getCurrentPosition() / ONE_CM_IN_PPR;
        }
    }
    
}
