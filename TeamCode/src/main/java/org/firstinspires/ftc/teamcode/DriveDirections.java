package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public abstract class DriveDirections extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontDrive = null;
    private DcMotor leftFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor leftBackDrive = null;
    private double moveSpeed = 0.3;

    public DcMotor armMotor = null;

    IntegratingGyroscope gyro;
    NavxMicroNavigationSensor navxMicro;

    double previousHeading = 0;
    double intergratedHeading = 0;



    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        leftFrontDrive = hardwareMap.get(DcMotor.class, "frontLeft");
        leftBackDrive = hardwareMap.get(DcMotor.class, "backLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "backRight");

        armMotor = hardwareMap.get(DcMotor.class, "nonExistentArm");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        armMotor.setDirection(DcMotor.Direction.FORWARD);

        //Calibrate NavX
        navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
        gyro = (IntegratingGyroscope)navxMicro;
        telemetry.addLine("Gyro Calibrating. Do Not Move!");
        telemetry.update();

        // Wait until the gyro calibration is complete
        while (navxMicro.isCalibrating())  {
            telemetry.addLine("calibrating...");
            telemetry.update();
            sleep(50);
        }
        telemetry.addLine("Gyro Calibrated. Press Start.");
        telemetry.update();

    }

        public void DriveInDirection(double power, String direction){
        switch(direction){
            case "FORWARD":
                rightFrontDrive.setPower(power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(power);
                leftBackDrive.setPower(power);
                break;

            case "BACKWARD":
                rightFrontDrive.setPower(-power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(-power);
                leftBackDrive.setPower(-power);
                break;

            case "LEFT":
                rightFrontDrive.setPower(power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(-power);
                leftBackDrive.setPower(power);
                break;

            case "RIGHT":
                rightFrontDrive.setPower(-power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(power);
                leftBackDrive.setPower(-power);
                break;

            case "STOP":
                rightFrontDrive.setPower(0);
                leftFrontDrive.setPower(0);
                rightBackDrive.setPower(0);
                leftBackDrive.setPower(0);
                break;
        }
    }

    public void DriveForDistance (double power, double distance, String direction){
        double clicksPerMeter = 2492.788;
        double targetClicks = distance*clicksPerMeter;
        double currentClicks = 0;

        double rightFrontClicks = 0;
        double leftFrontClicks = 0;
        double rightBackClicks = 0;
        double leftBackClicks = 0;

        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        DriveInDirection(power, direction);

        while(currentClicks < targetClicks){

            rightFrontClicks = Math.abs(rightFrontDrive.getCurrentPosition());
            leftFrontClicks = Math.abs(leftFrontDrive.getCurrentPosition());
            rightBackClicks = Math.abs(rightBackDrive.getCurrentPosition());
            leftBackClicks = Math.abs(leftBackDrive.getCurrentPosition());

            currentClicks = (rightFrontClicks+leftFrontClicks+rightBackClicks+leftBackClicks)/4;
        }
        DriveInDirection(0, "FORWARD");
    }

    public void DriveForTime(String direction, double power, double time){
        time*=1000;
        if(direction.equals("FORWARD")){
            rightFrontDrive.setPower(power);
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }else if(direction.equals("BACKWARD")){
            rightFrontDrive.setPower(-power);
            leftFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(-power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }else if(direction.equals("LEFT")){
            rightFrontDrive.setPower(power);
            leftFrontDrive.setPower(-power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }else if(direction.equals("RIGHT")){
            rightFrontDrive.setPower(-power);
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(-power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }else if(direction.equals("ROTATE_RIGHT")){
            rightFrontDrive.setPower(-power);
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }
        else if(direction.equals("ROTATE_LEFT")){
            rightFrontDrive.setPower(power);
            leftFrontDrive.setPower(-power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(-power);
            sleep((long)time);
            rightFrontDrive.setPower(0);
            leftFrontDrive.setPower(0);
            rightBackDrive.setPower(0);
            leftBackDrive.setPower(0);
        }
    }

    public void rotateToZAbs(double targetAngle, double power){
        double angle = getCumulativeZ();
        double difference = targetAngle - angle;
        if(difference > 0){
            while (angle < targetAngle) {
                angle = getCumulativeZ();
                rightFrontDrive.setPower(-power);
                leftFrontDrive.setPower(power);
                rightBackDrive.setPower(-power);
                leftBackDrive.setPower(power);

                telemetry.addLine("currentZ: " + getCurrentZ());
                telemetry.addLine("cumulativeZ: " + getCumulativeZ());
                telemetry.addLine("targetAngle: " + targetAngle);
                telemetry.addLine("rotation: counter clockwise");
                telemetry.update();
            }
        }

        else if(difference < 0){
            while (angle > targetAngle) {
                angle = getCumulativeZ();
                rightFrontDrive.setPower(power);
                leftFrontDrive.setPower(-power);
                rightBackDrive.setPower(power);
                leftBackDrive.setPower(-power);

                telemetry.addLine("currentZ" + getCurrentZ());
                telemetry.addLine("cumulativeZ" + getCumulativeZ());
                telemetry.addLine("targetAngle: " + targetAngle);
                telemetry.addLine("rotation: clockwise");
                telemetry.update();
            }
        }

        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);

    }
    //WIPPPPPPPP!!!!
    public void rotateToZLoc(double targetAngle, double power){
        double startAngle = getCumulativeZ();
        double angle = startAngle;
        double localAngle = startAngle;
        while (localAngle < targetAngle) {


            //rotate clockwise/right
            rightFrontDrive.setPower(-power);
            leftFrontDrive.setPower(power);
            rightBackDrive.setPower(-power);
            leftBackDrive.setPower(power);


            //telemetry
            telemetry.addLine("currentZ: " + getCurrentZ());
            telemetry.addLine("cumulativeZ: " + getCumulativeZ());
            telemetry.addLine("targetAngle: " + targetAngle);
            telemetry.addLine("rotation: counter clockwise");
            telemetry.update();
        }

        while (localAngle > targetAngle) {

            //rotate counter-clock/left
            rightFrontDrive.setPower(power);
            leftFrontDrive.setPower(-power);
            rightBackDrive.setPower(power);
            leftBackDrive.setPower(-power);

            //telemetry
            telemetry.addLine("currentZ" + getCurrentZ());
            telemetry.addLine("cumulativeZ" + getCumulativeZ());
            telemetry.addLine("targetAngle: " + targetAngle);
            telemetry.addLine("rotation: clockwise");
            telemetry.update();
        }


        rightFrontDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftBackDrive.setPower(0);

    }

//    public void straightDrive(String direction, double power, double dist, double errorThresh, double powerDifference){
//        switch(direction){
//            case "FORWARD":
//
//                break;
//        }
//    }

    public double getCurrentZ() {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    public double getCumulativeZ() {
        double currentHeading = getCurrentZ();
        double deltaHeading = currentHeading - previousHeading;
        if (deltaHeading <= -180) {
            deltaHeading += 360;
        } else if (deltaHeading >= 180) {
            deltaHeading -= 360;
        }

        intergratedHeading += deltaHeading;
        previousHeading = currentHeading;

        return  intergratedHeading;
    }

    //Test code for arm
    //Gets height in millimeters
    public double getArmHeight() {
        return armMotor.getCurrentPosition()/3.433;
    }
    //distance is in millimeters
    public void armToHeight (double power, double height){
        double currentHeight = getArmHeight();

        if (height>currentHeight) {
            armMotor.setPower(power);
            while (currentHeight < height) {
                currentHeight = getArmHeight();
            }
        } else if (height<currentHeight) {
            armMotor.setPower(-power);
            while (currentHeight > height) {
                currentHeight = getArmHeight();
            }
        }
        armMotor.setPower(0);
    }



}

