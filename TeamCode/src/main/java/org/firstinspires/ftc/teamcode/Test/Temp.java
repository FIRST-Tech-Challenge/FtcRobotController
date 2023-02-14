package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="OneConeRight")
public class Temp extends LinearOpMode {



    public void runOpMode() throws InterruptedException {







        VisionRight detector = new VisionRight(this);





        waitForStart();

        telemetry.addData("Red", detector.one);
        telemetry.addData("Green", detector.two);
        telemetry.addData("Blue", detector.three);
        telemetry.update();

        //pos strafe = left strafing
        //high =
        //mid =
        //low =




        if (detector.one == true) {


            gyroDrive(DRIVE_SPEED06, 11, 11, 11, 11, 0);
            drivebackleftandfrontright(32,DRIVE_SPEED06);
            gyroTurn(TURN_SPEED, 88);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-10,-10,-10,-10,88);
            golift(31,.6);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-36,-36,-36,-36,88);
            gyroTurn(TURN_SPEED, 53);
            gyroHold(TURN_SPEED_FIX,53, TIME);
            gyroDrive(DRIVE_SPEED07,-13.8,-13.8,-13.8,-13.8,53);


            //drop cone at high 1
            letgogirl();
            gyroDrive(DRIVE_SPEED07,13,13,13,13,41);
            gyroTurn(TURN_SPEED, 88);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,6,6,6,6,0);

//            gyroDrive(DRIVE_SPEED07,10.8,10.8,10.8,10.8,39);
//            gyroHold(TURN_SPEED_FIX,41, TIME);
//            golift(-19, .6);
//            gyroTurn(TURN_SPEED,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroDrive(DRIVE_SPEED08, -56, -56, -56, -56,0);
//            gyroDrive(DRIVE_SPEED05, -5, -5, -5, -5,0);
//            findline(0.5);
//            golift(-6,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 22, 22, 22, 22,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroTurn(TURN_SPEED, 142.5);
//            gyroHold(TURN_SPEED_FIX,142.5, TIME);
//
//            gyroDrive(DRIVE_SPEED07,-13.5, -13.5,-13.5,-13.5,142.5);
////
////            //drop cone at high 2
//            letgogirl();
//
//            gyroDrive(DRIVE_SPEED07,13.5,13.5,13.5,13.5,142);
//            golift(-19,.7);
//            gyroTurn(TURN_SPEED,4);
//            gyroHold(TURN_SPEED_FIX,4, TIME);
//            gyroDrive(DRIVE_SPEED07, -24,-24, -24, -24,4);
//
//            golift(-9,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 23, 23, 23, 23,0);
//
//            gyroTurn(TURN_SPEED, 145);
//            gyroHold(TURN_SPEED_FIX,145, .5);
//            gyroDrive(DRIVE_SPEED07,-12,-12,-12,-12,145);
//
//            letgogirl();
//            gyroDrive(DRIVE_SPEED08,12,12,12,12,140);
//            golift(-27,1);
//            gyroTurn(1, -180);
//            gyroDrive(1,-25,-25,-25,-25,-180);
//
//
//
//
        } else if (detector.two == true) {
            gyroDrive(DRIVE_SPEED06, 11, 11, 11, 11, 0);
            drivebackleftandfrontright(32,DRIVE_SPEED06);
            gyroTurn(TURN_SPEED, 88);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-10,-10,-10,-10,88);
            golift(31,.6);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-36,-36,-36,-36,88);
            gyroTurn(TURN_SPEED, 53);
            gyroHold(TURN_SPEED_FIX,53, TIME);
            gyroDrive(DRIVE_SPEED07,-13.8,-13.8,-13.8,-13.8,53);


            //drop cone at high 1
            letgogirl();
            gyroDrive(DRIVE_SPEED07,13,13,13,13,41);
            gyroTurn(TURN_SPEED, 0);
            gyroHold(TURN_SPEED_FIX,0, TIME);
            gyroDrive(DRIVE_SPEED07,-24,-24,-24,-24,0);

//            gyroDrive(DRIVE_SPEED07,11,11,11,11,0);
//            drivebackleftandfrontright(32,DRIVE_SPEED05);
//            gyroTurn(TURN_SPEED, 88);
//            gyroHold(TURN_SPEED_FIX,88, TIME);
//            gyroDrive(DRIVE_SPEED07,-13,-13,-13,-13,88);
//            golift(31,1);
//            gyroHold(TURN_SPEED_FIX,88, TIME);
//            gyroDrive(DRIVE_SPEED07,-34,-34,-34,-34,88);
//            gyroTurn(TURN_SPEED, 39.5);
//            gyroHold(TURN_SPEED_FIX,39, TIME);
//            gyroDrive(DRIVE_SPEED07,-10,-10,-10,-10,39);
//
//
////            //drop cone at high 1
//            letgogirl();
//            gyroDrive(DRIVE_SPEED07,10.3,10.3,10.3,10.3,39);
//            gyroHold(TURN_SPEED_FIX,41, TIME);
//            golift(-19, .6);
//            gyroTurn(TURN_SPEED,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroDrive(DRIVE_SPEED08, -56, -56, -56, -56,0);
//            gyroDrive(DRIVE_SPEED05, -5, -5, -5, -5,0);
//            findline(0.5);
//            golift(-6,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 22, 22, 22, 22,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroTurn(TURN_SPEED, 142.5);
//            gyroHold(TURN_SPEED_FIX,142.5, TIME);
//
//            gyroDrive(DRIVE_SPEED07,-13.5, -13.5,-13.5,-13.5,142.5);
////
////            //drop cone at high 2
//            letgogirl();
//
//            gyroDrive(DRIVE_SPEED07,13.5,13.5,13.5,13.5,142);
//            golift(-19,.7);
//            gyroTurn(TURN_SPEED,4);
//            gyroHold(TURN_SPEED_FIX,4, TIME);
//            gyroDrive(DRIVE_SPEED07, -24,-24, -24, -24,4);
//
//            golift(-9,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 23, 23, 23, 23,0);
//
//            gyroTurn(TURN_SPEED, 145);
//            gyroHold(TURN_SPEED_FIX,145, .5);
//            gyroDrive(DRIVE_SPEED07,-12,-12,-12,-12,145);
//
//            letgogirl();
//            gyroDrive(1,23,23,23,23,88);
//
        }
//
        else if (detector.three == true) {



            gyroDrive(DRIVE_SPEED06, 11, 11, 11, 11, 0);
            drivebackleftandfrontright(32,DRIVE_SPEED06);
            gyroTurn(TURN_SPEED, 88);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-10,-10,-10,-10,88);
            golift(31,.6);
            gyroHold(TURN_SPEED_FIX,88, TIME);
            gyroDrive(DRIVE_SPEED07,-36,-36,-36,-36,88);
            gyroTurn(TURN_SPEED, 53);
            gyroHold(TURN_SPEED_FIX,53, TIME);
            gyroDrive(DRIVE_SPEED07,-13.8,-13.8,-13.8,-13.8,53);


            //drop cone at high 1
            letgogirl();
            gyroDrive(DRIVE_SPEED07,13,13,13,13,41);
            gyroTurn(TURN_SPEED, 0);
            gyroHold(TURN_SPEED_FIX,0, TIME);
            gyroDrive(DRIVE_SPEED07,-55,-55,-55,-55,0);
//
//            gyroDrive(DRIVE_SPEED07,11,11,11,11,0);
//            drivebackleftandfrontright(32,DRIVE_SPEED05);
//            gyroTurn(TURN_SPEED, 88);
//            gyroHold(TURN_SPEED_FIX,88, TIME);
//            gyroDrive(DRIVE_SPEED07,-13,-13,-13,-13,88);
//            golift(31,1);
//            gyroHold(TURN_SPEED_FIX,88, TIME);
//            gyroDrive(DRIVE_SPEED07,-34,-34,-34,-34,88);
//            gyroTurn(TURN_SPEED, 39.5);
//            gyroHold(TURN_SPEED_FIX,39, TIME);
//            gyroDrive(DRIVE_SPEED07,-10.5,-10.5,-10.5,-10.5,39);
//
//
////            //drop cone at high 1
//            letgogirl();
//            gyroDrive(DRIVE_SPEED07,10.8,10.8,10.8,10.8,39);
//            gyroHold(TURN_SPEED_FIX,41, TIME);
//            golift(-19, .6);
//            gyroTurn(TURN_SPEED,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroDrive(DRIVE_SPEED08, -56, -56, -56, -56,0);
//            gyroDrive(DRIVE_SPEED05, -5, -5, -5, -5,0);
//            findline(1);
//            golift(-6,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 22, 22, 22, 22,0);
//            gyroHold(TURN_SPEED_FIX,0, TIME);
//            gyroTurn(TURN_SPEED, 142.5);
//            gyroHold(TURN_SPEED_FIX,142.5, TIME);
//
//            gyroDrive(DRIVE_SPEED07,-13.5, -13.5,-13.5,-13.5,142.5);
////
////            //drop cone at high 2
//            letgogirl();
//
//            gyroDrive(DRIVE_SPEED07,13.5,13.5,13.5,13.5,142);
//            golift(-19,.7);
//            gyroTurn(TURN_SPEED,4);
//            gyroHold(TURN_SPEED_FIX,4, TIME);
//            gyroDrive(DRIVE_SPEED07, -24,-24, -24, -24,4);
//
//            golift(-9,.7);
//            getitgirl();
//            golift(26, .9);
//            gyroDrive(DRIVE_SPEED07, 23, 23, 23, 23,0);
//
//            gyroTurn(TURN_SPEED, 145);
//            gyroHold(TURN_SPEED_FIX,145, .5);
//            gyroDrive(DRIVE_SPEED07,-12,-12,-12,-12,145);
//
//            letgogirl();
//            gyroDrive(DRIVE_SPEED08,12,12,12,12,140);
//            golift(-27,1);
//            gyroTurn(1, -15);
//            gyroDrive(1,-25,-25,-25,-25,-15);
//
//
//
        }


    }











    public void rangeDrive(double speed, double backDistance) {

        speed = Range.clip(Math.abs(speed), 0.0, 1.0);

//        if (rangebrothers.getDistance(DistanceUnit.INCH) >= backDistance) {
        while (rangebrothers.getDistance(DistanceUnit.INCH) < backDistance) {
            if (Math.abs(rangebrothers.getDistance(DistanceUnit.INCH) - backDistance) > MAX_ACCEPTABLE_ERROR) {
                if (!rangeCheck(rangebrothers, backDistance)) {
                    break;
                }
            }
            frontleft.setPower(speed);
            backleft.setPower(speed);
            frontright.setPower(speed);
            backright.setPower(speed);

            telemetry.addData("Sensor Back Distance: ", rangebrothers.getDistance(DistanceUnit.INCH));
            telemetry.addData("Target Back Distance: ", backDistance);
            telemetry.addLine("Moving Backwards");
            telemetry.update();
        }
//            while (rangebrothers.getDistance(DistanceUnit.INCH) > backDistance) {
//                if (Math.abs(rangebrothers.getDistance(DistanceUnit.INCH) - backDistance) > MAX_ACCEPTABLE_ERROR) {
//                    if (!rangeCheck(rangebrothers, backDistance)) {
//                        break;
//                    }
//                }
//                frontleft.setPower(-speed);
//                backleft.setPower(-speed);
//                frontright.setPower(-speed);
//                backright.setPower(-speed);
//
//                telemetry.addData("Sensor Back Distance: ", rangebrothers.getDistance(DistanceUnit.INCH));
//                telemetry.addData("Target Back Distance: ", backDistance);
//                telemetry.addLine("Moving Forwards");
//                telemetry.update();
//            }
        frontleft.setPower(0);
        backleft.setPower(0);
        frontright.setPower(0);
        backright.setPower(0);


    }



    private boolean rangeCheck(ModernRoboticsI2cRangeSensor range_sensor, double desired_distance){
        final int TRIES = 3;
        for (int i = 0; i < TRIES; i++){
            if (Math.abs(range_sensor.getDistance(DistanceUnit.INCH) - desired_distance) < MAX_ACCEPTABLE_ERROR){
                return true;
            }
            telemetry.addData("TRY ",i);
            telemetry.addData("Range Value: ", range_sensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("Target: ", desired_distance);
            telemetry.update();
            try {
                Thread.sleep(1500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
        return false;
    }






    public void sensewall1(){
        while (rangebrothers.getDistance(DistanceUnit.CM) <= 85) {
            backright.setPower(.3);
            backleft.setPower(.3);
            frontright.setPower(.3);
            frontleft.setPower(.3);

        }
    }

    public void sensewall2(){
        while (rangebrothers.getDistance(DistanceUnit.CM) <= 52) {
            backright.setPower(.3);
            backleft.setPower(.3);
            frontright.setPower(.3);
            frontleft.setPower(.3);

        }
        backright.setPower(0);
        backleft.setPower(0);
        frontright.setPower(0);
        frontleft.setPower(0);
    }




    public void moveToPosition(double inches, double speed){
        //
        int move = (int)(Math.round(inches*conversion));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() + move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() + move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()){
            if (exit){
                frontright.setPower(0);
                frontleft.setPower(0);
                backright.setPower(0);
                backleft.setPower(0);
                return;
            }
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        return;


    }


















//    public void turnWithGyro(double degrees, double speedDirection) {
//        //<editor-fold desc="Initialize">
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        double yaw = -angles.firstAngle;//make this negative
//        telemetry.addData("Speed Direction", speedDirection);
//        telemetry.addData("Yaw", yaw);
//        telemetry.update();
//        //
//        telemetry.addData("stuff", speedDirection);
//        telemetry.update();
//        //
//        double first;
//        double second;
//        //</editor-fold>
//        //
//        if (speedDirection > 0) {//set target positions
//            //<editor-fold desc="turn right">
//            if (degrees > 10) {
//                first = (degrees - 10) + devertify(yaw);
//                second = degrees + devertify(yaw);
//            } else {
//                first = devertify(yaw);
//                second = degrees + devertify(yaw);
//            }
//            //</editor-fold>
//        } else {
//            //<editor-fold desc="turn left">
//            if (degrees > 10) {
//                first = devertify(-(degrees - 10) + devertify(yaw));
//                second = devertify(-degrees + devertify(yaw));
//            } else {
//                first = devertify(yaw);
//                second = devertify(-degrees + devertify(yaw));
//            }
//            //
//            //</editor-fold>
//        }
//        //
//        //<editor-fold desc="Go to position">
//        Double firsta = convertify(first - 5);//175
//        Double firstb = convertify(first + 5);//-175
//        //
//        turnWithEncoder(speedDirection);
//        //
//        if (Math.abs(firsta - firstb) < 11) {
//            while (!(firsta < yaw && yaw < firstb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        } else {
//            //
//            while (!((firsta < yaw && yaw < 180) || (-180 < yaw && yaw < firstb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("first before", first);
//                telemetry.addData("first after", convertify(first));
//                telemetry.update();
//            }
//        }
//        //
//        Double seconda = convertify(second - 5);//175
//        Double secondb = convertify(second + 5);//-175
//        //
//        turnWithEncoder(speedDirection / 3);
//        //
//        if (Math.abs(seconda - secondb) < 11) {
//            while (!(seconda < yaw && yaw < secondb) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            while (!((seconda < yaw && yaw < 180) || (-180 < yaw && yaw < secondb)) && opModeIsActive()) {//within range?
//                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//                gravity = imu.getGravity();
//                yaw = -angles.firstAngle;
//                telemetry.addData("Position", yaw);
//                telemetry.addData("second before", second);
//                telemetry.addData("second after", convertify(second));
//                telemetry.update();
//            }
//            frontleft.setPower(0);
//            frontright.setPower(0);
//            backleft.setPower(0);
//            backright.setPower(0);
//        }
//        //</editor-fold>
//        //
//        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//    }

    //
    /*
    This function uses the encoders to strafe left or right.
    Negative input for inches results in left strafing.
     */
    public void strafeToPosition(double inches, double speed) {
        //
        int move = (int) (Math.round(inches * cpi * meccyBias));
        //
        backleft.setTargetPosition(backleft.getCurrentPosition() - move);
        frontleft.setTargetPosition(frontleft.getCurrentPosition() + move);
        backright.setTargetPosition(backright.getCurrentPosition() + move);
        frontright.setTargetPosition(frontright.getCurrentPosition() - move);
        //
        frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //
        frontleft.setPower(speed);
        backleft.setPower(speed);
        frontright.setPower(speed);
        backright.setPower(speed);
        //
        while (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy()) {
        }
        frontright.setPower(0);
        frontleft.setPower(0);
        backright.setPower(0);
        backleft.setPower(0);
        return;
    }

    //
    /*
    These functions are used in the turnWithGyro function to ensure inputs
    are interpreted properly.
     */
    public double devertify(double degrees) {
        if (degrees < 0) {
            degrees = degrees + 360;
        }
        return degrees;
    }

    public double convertify(double degrees) {
        if (degrees > 179) {
            degrees = -(360 - degrees);
        } else if (degrees < -180) {
            degrees = 360 + degrees;
        } else if (degrees > 360) {
            degrees = degrees - 360;
        }
        return degrees;
    }

    //
    /*
    This function is called at the beginning of the program to activate
    the IMU Integrated Gyro.
     */
//    public void initGyro() {
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        //parameters.calibrationDataFile = "GyroCal.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        //
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//    }

    //
    /*
    This function is used in the turnWithGyro function to set the
    encoder mode and turn.
     */
    public void turnWithEncoder(double input) {
        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //
        frontleft.setPower(input);
        backleft.setPower(input);
        frontright.setPower(-input);
        backright.setPower(-input);
    }

}
