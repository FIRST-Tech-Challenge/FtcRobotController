//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDriveTest01", group = "TeleOp")
public class TankDriveTest01 extends LinearOpMode {

    //Motors
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorDuck;
    DcMotor motorArm;
    Servo servoGrabber;
    Servo servoArm;
    double x = 0.7;

    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        //Initialize
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorArm = hardwareMap.dcMotor.get("motorArm");
        motorDuck = hardwareMap.dcMotor.get("motorDuck");
        servoGrabber = hardwareMap.servo.get("servoGrabber");
        servoArm = hardwareMap.servo.get("servoArm");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorDuck.setDirection(DcMotorSimple.Direction.REVERSE);
        servoGrabber.setPosition(0.3);
        servoArm.setPosition(0.4);
        int position = 0;
        boolean ispressed=false;
        double motorPower = 0.4;
        double increase = 1;
        double oldPosition = 0;

         /* DISABLE ENCODER
         motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorDuck.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorArm.setTargetPosition(0);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu". Certain parameters must be specified before using the imu.
         /*
         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
         parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
         parameters.loggingEnabled = true;
         parameters.loggingTag = "IMU";
         imu = hardwareMap.get(BNO055IMU.class, "imu");
         imu.initialize(parameters);*/

        waitForStart();

        while (opModeIsActive()) {
            motorBackLeft.setPower((gamepad1.left_stick_y));
            motorFrontLeft.setPower((gamepad1.left_stick_y));
            motorBackRight.setPower((gamepad1.left_stick_y));
            motorFrontRight.setPower((gamepad1.left_stick_y));

            motorBackRight.setPower((gamepad1.right_stick_x));
            motorFrontRight.setPower((gamepad1.right_stick_x));
            motorBackLeft.setPower(-(gamepad1.right_stick_x));
            motorFrontLeft.setPower(-(gamepad1.right_stick_x));

            if (position==6){
                motorArm.setTargetPosition(-1650);
                servoArm.setPosition(0.9);
                motorArm.setPower(motorPower);
            }

            if (position==5){
                motorArm.setTargetPosition(-1475);
                servoArm.setPosition(0.8);
                motorArm.setPower(motorPower);
            }
            if(position==4) {
                    motorArm.setTargetPosition(-1275);
                    servoArm.setPosition(0.6);
                    motorArm.setPower(motorPower);
            }
            if(position==3) {
                motorArm.setTargetPosition(-900);
                motorArm.setPower(0.4);
            }
            if(position==2) {
                    motorArm.setTargetPosition(-500);
                    servoArm.setPosition(0.7);
                    motorArm.setPower(motorPower);
            }

            else if (position==1) {
                motorArm.setTargetPosition(-260);
                servoArm.setPosition(0.6);
                motorArm.setPower(motorPower);
            }

            else if (position==0) {
                motorArm.setTargetPosition(0);
                servoArm.setPosition(0.4);
                motorArm.setPower(motorPower);
            }
            if (gamepad1.dpad_up){
                if (!ispressed) {
                    if (position == 3) {
                        motorPower = 0.15;
                    }
                    position += increase;
                }
                ispressed=true;
            }
            if (!gamepad1.dpad_up && !gamepad1.dpad_down){
                ispressed=false;
            }
            else if (gamepad1.dpad_down){
                if (!ispressed) {
                    if (position == 3) {
                        motorPower = 0.15;
                    }
                    position -= increase;
                }
                ispressed=true;
            }
            else {
                motorPower = 0.5;
            }
            if (position<0){
                position=0;
            }
            if (position>6){
                position=6;
            }


            if(gamepad1.x) {
                servoGrabber.setPosition(0.3);
            }

            else if(gamepad1.a) {
                servoGrabber.setPosition(0.7);
            }

            if (gamepad1.right_bumper) {
                while (true) {
                    motorDuck.setPower(x);
                    pauseMillis(150);
                    x += 0.05;
                    telemetry.addData("duckPower", motorDuck.getPower());
                    telemetry.update();
                    if (x>=0.85){
                        pauseMillis(1000);
                        motorDuck.setPower(-.1);
                        pauseMillis(30);
                        motorDuck.setPower(0);
                        x=0.7;
                        break;
                    }
                }
            }

        }
    }

    public void pauseMillis(double time) {
        double startTime = System.currentTimeMillis();
        while (System.currentTimeMillis() - startTime < time && opModeIsActive()) {
            idle();
        }
    }
}