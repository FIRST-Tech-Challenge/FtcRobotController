//package org.firstinspires.ftc.blackswan.poopoogarabge;
//
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import static java.lang.Math.abs;
//
//import org.firstinspires.ftc.blackswan.Robot;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
//@TeleOp(name="TeleopICKY")
//
//public class Teleop extends LinearOpMode {
//    BNO055IMU imu;
//    double MAX_SPEED = 0.9;
//    Robot robot;
//    final int TICKS_PER_ROTATION = 537;
//
//    int GyroSensorVariable = 1;
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//
//
//        robot = new Robot(hardwareMap, telemetry, this);
//
//        DcMotor frontLeft, backLeft, frontRight, backRight, arm, carousel, intake;
//
//        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
//        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
//        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
//        backRight = hardwareMap.get(DcMotor.class, "backRight");
//
//        carousel = hardwareMap.get(DcMotor.class, "carousel");
//
//        arm = hardwareMap.get(DcMotor.class, "arm");
//
//        intake = hardwareMap.get(DcMotor.class, "intake");
//
//        frontLeft.setDirection(DcMotor.Direction.FORWARD);
//        frontRight.setDirection(DcMotor.Direction.REVERSE);
//        backLeft.setDirection(DcMotor.Direction.FORWARD);
//        backRight.setDirection(DcMotor.Direction.REVERSE);
//        arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        //    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
////        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
////        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
////        parameters.calibrationDataFile = "AdafruitIMUCalibration.json"; // see the calibration sample opmode
////        parameters.loggingEnabled      = true;
////        parameters.loggingTag          = "IMU";
////        parameters.accelerationIntegrationAlgorithm=null;//= new JustLoggingAccelerationIntegrator();
////        imu = hardwareMap.get(BNO055IMU.class, "imu");
////        imu.initialize(parameters);
//        imu = IMUstorage.getImu(hardwareMap, telemetry);
//
//        waitForStart();
//
//        //telemetry testing delete later!!!
//        String detection = "none";
//        while (opModeIsActive()) {
//            Orientation angles = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            //turn with right stick
////            telemetry.addData("left stick value x", gamepad1.left_stick_x);
////            telemetry.addData("left stick value y", gamepad1.left_stick_y);
//            telemetry.addData("left stick value x", gamepad2.left_stick_x);
//            telemetry.addData("left stick value y", gamepad2.left_stick_y);
//            telemetry.addData("detection", detection);
//            telemetry.addData("gyrovar", GyroSensorVariable);
//            telemetry.addData("angle" , angles.secondAngle);
//            telemetry.update();
//            //sensorvar 1 = facing forward 2= facing backward 3 = facing left 4= facing right
//            if(angles.secondAngle > -45 && angles.secondAngle < 45){
//                GyroSensorVariable = 1;
//            }else if(angles.secondAngle < -45 && angles.secondAngle > -135){
//                GyroSensorVariable = 3;
//            }else if(angles.secondAngle > 45 && angles.secondAngle < 135){
//                GyroSensorVariable = 4;
//            }else{
//                GyroSensorVariable = 2;
//            }
//            if (gamepad1.right_stick_x > 0.1) {
//                //Turn Right
//                telemetry.addData("positive", gamepad1.right_stick_x);
//                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
//                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
//                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
//                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
//            } else if (gamepad1.right_stick_x < -0.1) {
//                //Turn Left
//                telemetry.addData("negative", gamepad1.right_stick_x);
//                frontLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
//                frontRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
//                backLeft.setPower(gamepad1.right_stick_x * MAX_SPEED);
//                backRight.setPower(gamepad1.right_stick_x * MAX_SPEED * -1);
//            } else if (gamepad1.left_stick_y > 0.1) {
//                if(GyroSensorVariable == 1) {
//                    //move Down (Normal)
//                    frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    detection = "Down";
//                }else if (GyroSensorVariable == 2){
//                    //move Down orientation down
//                    frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    backRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    detection = "Down O d";
//                }else if(GyroSensorVariable == 3){
//                    //move down orientation left
//                    frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    detection = "Down O l";
//                }else if(GyroSensorVariable == 4){
//                    //move Down orientation right
//                    frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    backRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    detection = "Down O r";
//                }
//            } else if (gamepad1.left_stick_y < -0.1) {
//                if(GyroSensorVariable == 1) {
//                    //move Up (Normal)
//                    frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    detection = "Up";
//                }else if (GyroSensorVariable == 2){
//                    //move Up in orientation down
//                    frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    backRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    detection = "Up O d";
//                }else if(GyroSensorVariable == 3){
//                    //move Up orientation left
//                    frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    backRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    detection = "Up O l";
//                }else if(GyroSensorVariable == 4){
//                    //move Up orientation right
//                    frontLeft.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    backRight.setPower(gamepad1.left_stick_y * MAX_SPEED * -1);
//                    frontRight.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    backLeft.setPower(gamepad1.left_stick_y * MAX_SPEED);
//                    detection = "Up O r";
//                }
//            } else if (gamepad1.left_stick_x > 0.1) {
//                if (GyroSensorVariable == 2) {
//                    //move Right orientation down
//                    frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    backRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    detection = "Right O d";
//                } else if (GyroSensorVariable == 1) {
//                    //move Right (Normal)
//                    frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    detection = "Right";
//                }else if(GyroSensorVariable == 3){
//                    //move Right orientation left
//                    frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    backRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    detection = "Right O l";
//                }else if(GyroSensorVariable == 4){
//                    //move Right orientation right
//                    frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    detection = "Right O r";
//                }
//
//            } else if (gamepad1.left_stick_x < -0.1) {
//                if (GyroSensorVariable == 1) {
//                    //move Left (Normal)
//                    frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    detection = "Left";
//                } else if (GyroSensorVariable == 2) {
//                    //move Left orientation down
//                    frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    backRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    detection = "Left O d";
//                }else if(GyroSensorVariable == 3){
//                    //move Left orientation left
//                    frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    backRight.setPower(gamepad1.left_stick_x * MAX_SPEED);
//                    detection = "Left O l";
//                }else if(GyroSensorVariable == 4){
//                    //move Left orientation right
//                    frontLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    frontRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    backLeft.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    backRight.setPower(gamepad1.left_stick_x * MAX_SPEED * -1);
//                    detection = "Left O r";
//                }
//
//            } else {
//                frontLeft.setPower(0);
//                frontRight.setPower(0);
//                backLeft.setPower(0);
//                backRight.setPower(0);
//                detection = "None";
//            }
//            if (gamepad2.dpad_up) { //up
//                arm.setTargetPosition(1250);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(.5);
//
//            }
//            if (gamepad2.dpad_left) { //middle
//                arm.setTargetPosition(825);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(.5);
//
//            }
//            if (gamepad2.dpad_right) { //low
//                arm.setTargetPosition(500);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(.5);
//
//            }
//
//            if (gamepad2.dpad_down) {
//                arm.setTargetPosition(75);
//                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                arm.setPower(.3);
////                while (arm.isBusy() && opModeIsActive()) {
////
////                }
////                arm.setPower(0);
////                arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//            }
//
//            if (gamepad2.right_trigger > 0.1) {
//                intake.setPower(-1);
//            } else if (gamepad2.left_trigger > 0.1) {
//                intake.setPower(1);
//            } else {
//                intake.setPower(0);
//            }
//
//
////this is to set the intake to cover the element inside
////            if (gamepad2.a) {
////                intake.setTargetPosition(-356);
////                intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
////                intake.setPower(0.8);
////                while (intake.isBusy() && opModeIsActive()) {
////                }
////
////                telemetry.update();
////            }
//
//            turnDuck(carousel);
//
//
//
//
//
//        }
//    }
//
//    protected void turnDuck(DcMotor carousel){
//        if(gamepad2.right_bumper){
//            carousel.setPower(-0.5 );
//        } else  if (gamepad2.left_bumper){
//            carousel.setPower(0.5);
//        }else {
//            carousel.setPower(0);
//        }
//    }
//
//
//}
//
