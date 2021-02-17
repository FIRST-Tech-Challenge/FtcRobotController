package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;


public class RobotClass {
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor shooterMotor;
    private DcMotor wobbleGoalRaise;
    private double ticks = 537;//537
  //  private CRServo continuous1;
    private Servo wobbleGoalGrippyThing;
    private CRServo shooterServo1;
    private CRServo shooterServo2;
    BNO055IMU imu;

    public Telemetry telemetry;
    ColorSensor colorSensor;

    LinearOpMode opmode;

    public RobotClass(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opmode) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft" );
        frontRight = hardwareMap.get(DcMotor.class, "frontRight" );
        backLeft = hardwareMap.get(DcMotor.class, "backLeft" );
        backRight = hardwareMap.get(DcMotor.class, "backRight" );
        shooterMotor = hardwareMap.get(DcMotor.class, "shooterMotor");
       // continuous1 = hardwareMap.get(CRServo.class, "cRServo1");
        wobbleGoalGrippyThing = hardwareMap.servo.get("wobbleGrip");
        shooterServo1 = hardwareMap.get(CRServo.class,"shooterServo1");
        wobbleGoalRaise = hardwareMap.dcMotor.get("wobbleLift");

       motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        this.telemetry = telemetry;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
       // parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm=null;//= new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu1");
        imu.initialize(parameters);

        this.opmode = opmode;


    }

    public void testGyro(){
        while(opmode.opModeIsActive()){
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES);
            telemetry.addData("gravity",imu.getGravity().toString());
            telemetry.addData("1",angles.firstAngle);
            telemetry.addData("2", angles.secondAngle);
            telemetry.addData("3", angles.thirdAngle);
            telemetry.update();
        }
    }
    public double getAngleFromGyro() {

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        return angles.firstAngle;
    }
//    public double relegateTargetAngle(double targetAngle) {
//        if (targetAngle > 180) {
//            double newTargetAngle = (targetAngle - 180);
//            targetAngle = -180+newTargetAngle;
//        }else if (targetAngle < -180) {
//            double newTargetAngle = (targetAngle + 180);
//            targetAngle = 180-newTargetAngle;
//        }
//        return targetAngle;
//    }

    public void forward (double speed, double rotations){
        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        telemetry.addData("Target Front Left Motor Position", leftCurrent);
        telemetry.addData("Target Front Right Motor Position", rightCurrent);
        telemetry.addData("Target Back Left Motor Position", backLeftCurrent);
        telemetry.addData("Target Back Right Motor Position", backRightCurrent);
        telemetry.update();
//        try {
//
//        } catch (InterruptedException e) {
//            e.printStackTrace();
//        }

        double toPositionLeft = leftCurrent + rotations*ticks;
        double toPositionRight = rightCurrent + rotations*ticks;
        double toPositionbackLeft = backLeftCurrent + rotations*ticks;
        double toPositionbackRight = backRightCurrent + rotations*ticks;

        telemetry.addData("Target Front Left Motor Position", toPositionLeft);
        telemetry.addData("Target Front Right Motor Position", toPositionRight);
        telemetry.addData("Target Back Left Motor Position", toPositionbackLeft);
        telemetry.addData("Target Front Left Motor Position", toPositionbackLeft);
        telemetry.update();

        frontLeft.setTargetPosition((int)toPositionLeft);
        frontRight.setTargetPosition((int)toPositionRight);
        backLeft.setTargetPosition((int)toPositionbackLeft);
        backRight.setTargetPosition((int)toPositionbackRight);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(speed));

        while (this.opmode.opModeIsActive() &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

            // Display it for the driver.
            motorTelemetry();
        }
        stopMotors();

        motorSetMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void backwards (double speed, double rotations) {
        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionLeft = leftCurrent - rotations * ticks;
        double toPositionRight = rightCurrent - rotations * ticks;
        double toPositionbackLeft = backLeftCurrent - rotations * ticks;
        double toPositionbackRight = backRightCurrent - rotations * ticks;

        frontLeft.setTargetPosition((int) toPositionLeft);
        frontRight.setTargetPosition((int) toPositionRight);
        backLeft.setTargetPosition((int) toPositionbackLeft);
        backRight.setTargetPosition((int) toPositionbackRight);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

//        telemetry.addData("Target Front Left Motor Position", toPositionLeft);
//        telemetry.addData("Target Front Right Motor Position", toPositionRight);
//        telemetry.addData("Target Back Left Motor Position", toPositionBackLeft);
//        telemetry.addData("Target Front Left Motor Position", toPositionLeft);
//        telemetry.update();
    }
        public void setSpeedForTurnRight (double speed) {
            frontLeft.setPower(speed);
            frontRight.setPower(-speed);
            backLeft.setPower(speed);
            backRight.setPower(-speed);
        }

        public void setSpeedForTurnLeft (double speed) {
            frontLeft.setPower(-speed);
            frontRight.setPower(speed);
            backLeft.setPower(-speed);
            backRight.setPower(speed);
        }

        public void stopMotors () {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
        }

        protected void motorTelemetry(){
            telemetry.addData("Target Front Left Motor Position", frontLeft.getCurrentPosition());
            telemetry.addData("Target Front Right Motor Position", frontRight.getCurrentPosition());
            telemetry.addData("Target Back Left Motor Position", backLeft.getCurrentPosition());
            telemetry.addData("Target Back Right Motor Position", backRight.getCurrentPosition());
            telemetry.update();
        }

//        public void turnRight (double speed, double angle) {
//        double anglemult = 1.5;
//
//            int leftCurrent = frontLeft.getCurrentPosition();
//            int rightCurrent = frontRight.getCurrentPosition();
//            int backLeftCurrent = backLeft.getCurrentPosition();
//            int backRightCurrent = backRight.getCurrentPosition();
//
//            double toPositionLeft = leftCurrent + anglemult  * angle;
//            double toPositionRight = rightCurrent - anglemult * angle;
//            double toPositionbackLeft = backLeftCurrent + anglemult * angle;
//            double toPositionbackRight = backRightCurrent - anglemult * angle;
//
//            frontLeft.setTargetPosition((int) toPositionLeft);
//            frontRight.setTargetPosition((int) toPositionRight);
//            backLeft.setTargetPosition((int) toPositionbackLeft);
//            backRight.setTargetPosition((int) toPositionbackRight);
//
//            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            frontLeft.setPower(speed);
//            frontRight.setPower(speed);
//            backLeft.setPower(speed);
//            backRight.setPower(speed);
//
//    }
//    public void turnLeft (double speed, double angle) {
//        double anglemult = 1.5;
//
//        int leftCurrent = frontLeft.getCurrentPosition();
//        int rightCurrent = frontRight.getCurrentPosition();
//        int backLeftCurrent = backLeft.getCurrentPosition();
//        int backRightCurrent = backRight.getCurrentPosition();
//
//        double toPositionLeft = leftCurrent - anglemult * angle;
//        double toPositionRight = rightCurrent + anglemult * angle;
//        double toPositionBackLeft = backLeftCurrent - anglemult * angle;
//        double toPositionBackRight = backRightCurrent + anglemult * angle;
//
//        frontLeft.setTargetPosition((int) toPositionLeft);
//        frontRight.setTargetPosition((int) toPositionRight);
//        backLeft.setTargetPosition((int) toPositionBackLeft);
//        backRight.setTargetPosition((int) toPositionBackRight);
//
//        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        frontLeft.setPower(speed);
//        frontRight.setPower(speed);
//        backLeft.setPower(speed);
//        backRight.setPower(speed);

//        telemetry.addData("Target Front Left Motor Position", toPositionLeft);
//        telemetry.addData("Target Front Right Motor Position", toPositionRight);
//        telemetry.addData("Target Back Left Motor Position", toPositionBackLeft);
//        telemetry.addData("Target Front Left Motor Position", toPositionLeft);
//        telemetry.update();
  //  }

    public void pivotRightSloppy (double speed, double angle) {
        setSpeedForTurnRight(speed);

        double targetAngle = getAngleFromGyro() - angle;

        while (getAngleFromGyro() > targetAngle && opmode.opModeIsActive()) {
            telemetry.addData("Gyro Angle: ", getAngleFromGyro());
            telemetry.update();
        }

        stopMotors();

        telemetry.addData("Gyro Angle", getAngleFromGyro());
        telemetry.update();
    }

    public void pivotLeftSloppy (double speed, double angle) {
        setSpeedForTurnLeft(speed);

        double targetAngle = getAngleFromGyro() + angle;

        while (getAngleFromGyro() < targetAngle && opmode.opModeIsActive()) {
            telemetry.addData("Gyro Angle: ", getAngleFromGyro());
            telemetry.update();
        }

        stopMotors();

        telemetry.addData("Gyro Angle", getAngleFromGyro());
        telemetry.update();
    }

    public void pivotRight (double speed, double angle) {
        double targetAngle = getAngleFromGyro() - angle;
        pivotRightSloppy(speed, angle);

        telemetry.addData("Middle Gyro Angle: ", getAngleFromGyro());
        telemetry.update();

        speed= speed*0.5;
        if (getAngleFromGyro()<targetAngle-0.5) {
            setSpeedForTurnLeft(speed);
            while (getAngleFromGyro() < targetAngle && opmode.opModeIsActive()) {
                telemetry.addData("Gyro Angle: ", getAngleFromGyro());
                telemetry.update();
            }
        }

        stopMotors();
        telemetry.addData("Completed Gyro Angle: ", getAngleFromGyro());
        telemetry.update();
    }
    public void pivotLeft (double speed, double angle) {
        double targetAngle = getAngleFromGyro() + angle;
        pivotLeftSloppy(speed, angle);

        telemetry.addData("Middle Gyro Angle: ", getAngleFromGyro());
        telemetry.update();

        speed= speed*0.5;
        setSpeedForTurnRight(speed);
        if (getAngleFromGyro()>targetAngle+0.5) {
            while (getAngleFromGyro() > targetAngle + 0.5 && opmode.opModeIsActive()) {
                telemetry.addData("Gyro Angle: ", getAngleFromGyro());
                telemetry.update();
            }
        }

        stopMotors();
        telemetry.addData("Completed Gyro Angle: ", getAngleFromGyro());
        telemetry.update();
    }
    public void forwardToWhite (double speed, double rotations, double speed2) throws InterruptedException {
        forward(speed,rotations);
        frontLeft.setPower(speed2);
        frontRight.setPower(speed2);
        backLeft.setPower(speed2);
        backRight.setPower(speed2);

        while (colorSensor.alpha() < 20) {
            
            telemetry.addData("Light Level: ", colorSensor.alpha());
            telemetry.update();
        }

        stopMotors();
    }
    public void forwardToBlue (double speed, double rotations, double speed2) {
        forward(speed,rotations);
        frontLeft.setPower(speed2);
        frontRight.setPower(speed2);
        backLeft.setPower(speed2);
        backRight.setPower(speed2);

        while (colorSensor.blue() < 20) {
            
            telemetry.addData("Blue Level: ", colorSensor.blue());
            telemetry.update();
        }

        stopMotors();
    }
    public void forwardToRed (double speed, double rotations, double speed2) throws InterruptedException {
        forward(speed,rotations);
        frontLeft.setPower(speed2);
        frontRight.setPower(speed2);
        backLeft.setPower(speed2);
        backRight.setPower(speed2);

        while (colorSensor.red() < 20) {
            
            telemetry.addData("Red Level: ", colorSensor.red());
            telemetry.update();
        }

        stopMotors();
    }
    public void mecanumWitchcraft (double degree, double time) {
        double x = java.lang.Math.cos(degree);
        double y = java.lang.Math.sin(degree);

        frontLeft.setPower(y + x);
        backLeft.setPower(y - x);
        frontRight.setPower(y - x);
        backRight.setPower(y + x);



        stopMotors();

    }
    public void mecanumWitchcraftColor (double degree, double rotations, int color) throws InterruptedException {
        double x = java.lang.Math.cos(degree);
        double y = java.lang.Math.sin(degree);

        frontLeft.setPower(y + x);
        backLeft.setPower(y - x);
        frontRight.setPower(y - x);
        backRight.setPower(y + x);

        if (color == 1) {
            while (colorSensor.alpha() < 20) {
                
            }
        } else if (color == 2)  {
            while (colorSensor.red() < 20) {
                
            }
        } else {
            while (colorSensor.blue() < 20) {
                
            }
        }
        stopMotors();
    }
    public void strafeLeft (double speed, double rotations) {

        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionLeft = leftCurrent + rotations*ticks;
        double toPositionRight = rightCurrent + rotations*ticks;
        double toPositionbackLeft = backLeftCurrent + rotations*ticks;
        double toPositionbackRight = backRightCurrent + rotations*ticks;

        frontLeft.setTargetPosition((int)toPositionLeft);
        frontRight.setTargetPosition((int)toPositionRight);
        backLeft.setTargetPosition((int)toPositionbackLeft);
        backRight.setTargetPosition((int)toPositionbackRight);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(-speed));
        frontRight.setPower(Math.abs(speed));
        backLeft.setPower(Math.abs(speed));
        backRight.setPower(Math.abs(-speed));

    }
    public void strafeRight (double speed, double rotations) {

        int leftCurrent = frontLeft.getCurrentPosition();
        int rightCurrent = frontRight.getCurrentPosition();
        int backLeftCurrent = backLeft.getCurrentPosition();
        int backRightCurrent = backRight.getCurrentPosition();

        double toPositionLeft = leftCurrent + rotations*ticks;
        double toPositionRight = rightCurrent + rotations*ticks;
        double toPositionbackLeft = backLeftCurrent + rotations*ticks;
        double toPositionbackRight = backRightCurrent + rotations*ticks;

        frontLeft.setTargetPosition((int)toPositionLeft);
        frontRight.setTargetPosition((int)toPositionRight);
        backLeft.setTargetPosition((int)toPositionbackLeft);
        backRight.setTargetPosition((int)toPositionbackRight);

        motorSetMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontLeft.setPower(Math.abs(speed));
        frontRight.setPower(Math.abs(-speed));
        backLeft.setPower(Math.abs(-speed));
        backRight.setPower(Math.abs(speed));
    }

    protected void motorSetMode(DcMotor.RunMode runMode){
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

//    public void testServo1 (double speed) {
//        continuous1.setPower(speed);
//
//        while (true){
//            //laughter
//        }
//
////        for (int num = 0; num < duration; num ++) {
////            //LAUGHTER
////            telemetry.addData("LAUGHTER: Now ", num);
////            telemetry.update();
//
//    }
    //Pretend "Engage" is actually ENGAGE!
    public void shooterEngage (double duration) {
        shooterMotor.setPower(1);

        for (int i=0; i < duration; i++ );


    }

    public void wobbleGoalGrippyThingGrab () {
        wobbleGoalGrippyThing.setPosition(.2);
    }
    public void wobbleGoalGrippyThingRelease () {
        wobbleGoalGrippyThing.setPosition(.9);
    }

    public void shooterServo1 (double speed) {
       shooterServo1.setPower(speed);
    }

    public void shooterServo2 (double speed) {
        shooterServo2.setPower(speed);
    }

    public void moveWobbleGoalArm (double speed, double rotation) {
        int currentPosition = frontLeft.getCurrentPosition();

        double toPosition = currentPosition + rotation*ticks;
        wobbleGoalRaise.setTargetPosition((int)toPosition);
        wobbleGoalRaise.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleGoalRaise.setPower(speed);
    }

    }
