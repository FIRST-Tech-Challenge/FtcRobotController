package org.firstinspires.ftc.teamcode.Qualifier_1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;


public class EncoderChassis {
    //initialize motor
    DcMotorEx motorLeftFront;
    DcMotorEx motorRightFront;
    DcMotorEx motorLeftBack;
    DcMotorEx motorRightBack;
    DcMotorEx ShooterMotor;
    DcMotorEx wobbleGoalMotor;
    Servo ShooterServo;

    // Initialize Encoder Variables
    final double robot_diameter = Math.sqrt(619.84);
    final double wheel_diameter = 3.93701;
    double[] encoder = new double[4];
    double xpos = 0;
    double ypos = 0;

    // these encoder variables vary depending on chassis type
    final double counts_per_motor_goBilda = 383.6;
    final double counts_per_inch =  (counts_per_motor_goBilda / (wheel_diameter * Math.PI));  //2*(counts_per_motor_goBilda / (wheel_diameter * Math.PI))
    final double counts_per_degree = counts_per_inch * robot_diameter * Math.PI / 360;

    /* local OpMode members. */
    private LinearOpMode op = null;
    private HardwareMap hardwareMap = null;
    private ElapsedTime period = null;

    private BNO055IMU imu;
    private Orientation lastAngles = null;
    private double globalAngle, power = .30, correction;
    private double IMUgain = 0.005;

    //set true to enable imu vice versa
    final boolean enableIMU = false;

    public EncoderChassis(LinearOpMode opMode) {

        op = opMode;
        hardwareMap = op.hardwareMap;

        period = new ElapsedTime();
        //lastAngles  = new Orientation();

        // Chassis motors
        motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");
        ShooterMotor = (DcMotorEx) hardwareMap.dcMotor.get("ShooterMotor");
        wobbleGoalMotor = (DcMotorEx) hardwareMap.dcMotor.get("wobbleGoalMotor");
        ShooterServo = (Servo) hardwareMap.servo.get("ShooterServo");

        // Chassis Motors
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);

        //Servo
        ShooterServo.setPosition(0);

        // reset encoder count kept by left motor.
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void stopAllMotors() {
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public double[] track(){
        double[] data = {0, 0, 0};
        double diff[]={motorLeftFront.getCurrentPosition() - encoder[0],motorRightFront.getCurrentPosition() - encoder[1],motorLeftBack.getCurrentPosition() - encoder[2],motorRightBack.getCurrentPosition()-encoder[3]};
        encoder[0] += diff[0];
        encoder[1] += diff[1];
        encoder[2] += diff[2];
        encoder[3] += diff[3];
        xpos += sqrt(2) * (-(diff[0]+diff[3])/(2*counts_per_inch) +  (diff[2]+diff[1])/(2*counts_per_inch));
        ypos += sqrt(2) * ((diff[0]+diff[3])/(2*counts_per_inch) + (diff[2]+diff[1])/(2*counts_per_inch));
        op.telemetry.addData("LeftFront",motorLeftFront.getCurrentPosition());
        op.telemetry.addData("RightFront",motorRightFront.getCurrentPosition());
        op.telemetry.addData("LeftBack",motorLeftBack.getCurrentPosition());
        op.telemetry.addData("RightBack",motorRightBack.getCurrentPosition());
        op.telemetry.addData("xpos",xpos);
        op.telemetry.addData("ypos",ypos);
        op.telemetry.addData("angle",getAngle());
        op.telemetry.update();
        data[0] = xpos;
        data[1] = ypos;
        data[2] = getAngle();
        return data;
    }
    public double getAngle() {
        double diff[]={motorLeftFront.getCurrentPosition() - encoder[0],motorRightFront.getCurrentPosition() - encoder[1],motorLeftBack.getCurrentPosition() - encoder[2],motorRightBack.getCurrentPosition()-encoder[3]};
        encoder[0] += diff[0];
        encoder[1] += diff[1];
        encoder[2] += diff[2];
        encoder[3] += diff[3];
        double angle = ((encoder[0]+encoder[2]) - (encoder[1]+encoder[3])) / (counts_per_inch *4* robot_diameter * Math.PI*360);
        angle %= 360;
        if (angle < -180) {
            angle += 360;
        }
        return angle;
    }
    public void turn(double target, double power) {
        double currentAngle = getAngle();
        int direction = 1;
        double difference = currentAngle - target;
        double targetAngle = target;
        if (target < 0) {
            direction = -1;
        }
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.5)) {
            currentAngle = getAngle();
            difference = targetAngle - currentAngle;
            if (difference * direction < 5) {
                power *= difference / 5;
                if (power < 0.2) {
                    power = 0.2 * power;
                }
            }
            motorRightBack.setPower(-power * direction);
            motorRightFront.setPower(-power * direction);
            motorLeftBack.setPower(power * direction);
            motorLeftFront.setPower(power * direction);
            op.telemetry.addData("current angle", currentAngle);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
    public void moveForward(double distance, double power) {
            double ticksToMove = counts_per_inch * distance;
            double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
            double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + ticksToMove;
            double newRightBackTargetPosition = motorRightBack.getCurrentPosition() + ticksToMove;
            double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
            motorLeftBack.setTargetPosition((int) newLeftBackTargetPosition);
            motorLeftFront.setTargetPosition((int) newLeftFrontTargetPosition);
            motorRightBack.setTargetPosition((int) newRightBackTargetPosition);
            motorRightFront.setTargetPosition((int) newRightFrontTargetPosition);

            op.telemetry.addData("ticks: ", (int) ticksToMove +
                    "LB: " + (int) newLeftBackTargetPosition + "LF: " + (int) newLeftFrontTargetPosition +
                    "RB: " + (int) newRightBackTargetPosition + "LB: " + (int) newRightFrontTargetPosition);
            op.telemetry.update();

            motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            motorRightFront.setPower(power);
            motorLeftFront.setPower(power);
            motorRightBack.setPower(power);
            motorLeftBack.setPower(power);

            while (op.opModeIsActive() && (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() &&
                    motorRightFront.isBusy())) {
                //correction = checkDirection();

                motorRightBack.setPower(power);
                motorRightFront.setPower(power);
                motorLeftBack.setPower(power);
                motorLeftFront.setPower(power);
//            op.telemetry.addData("correction", correction);
//            op.telemetry.update();
//            op.idle();
            }

            stopAllMotors();
            ypos+=48;
            // Changes motor mode back to default
            motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    public void moveSideways(double distance, double power) {
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() - ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() - ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        motorLeftBack.setTargetPosition((int) newLeftBackTargetPosition);
        motorLeftFront.setTargetPosition((int) newLeftFrontTargetPosition);
        motorRightBack.setTargetPosition((int) newRightBackTargetPosition);
        motorRightFront.setTargetPosition((int) newRightFrontTargetPosition);

        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        op.telemetry.addData("ticks: ", (int) ticksToMove +
                "LB: " + (int) newLeftBackTargetPosition + "LF: " + (int) newLeftFrontTargetPosition +
                "RB: " + (int) newRightBackTargetPosition + "LB: " + (int) newRightFrontTargetPosition);
        op.telemetry.update();

        motorLeftBack.setPower(power);
        motorRightBack.setPower(-power);
        motorLeftFront.setPower(-power);
        motorRightFront.setPower(power);

        while (op.opModeIsActive() && (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() &&
                motorRightFront.isBusy())) {

            motorRightBack.setPower(power - correction);
            motorRightFront.setPower(power + correction);
            motorLeftBack.setPower(power - correction);
            motorLeftFront.setPower(power + correction);
        }

        stopAllMotors();

        // Changes motor mode back to default
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void moveAngle(double x, double y, double power) {
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double startAngle =getAngle();
        double[] currentPosition = track();
        double[] target_position = {0, 0, 0};
        double anglecorrection;
        target_position[0] = currentPosition[0] + x;
        target_position[1] = currentPosition[1] + y;
        target_position[2] = currentPosition[2];
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        double angleInRadians = atan2(y, x) - getAngle() * PI / 180;
        double[] anglePower = {sin(angleInRadians + PI / 4), sin(angleInRadians - PI / 4)};
        /*try {
            //Create File
            File myFTCfile = new File("/storage/emulated/0/tmp/OdometryTest.csv");
            if (myFTCfile.createNewFile()) {
                op.telemetry.addData("moveAngleOdometry:", "File created:%S\n", "Odomeytry" );
                op.telemetry.update();
            } else {
                op.telemetry.addData("moveAngleOdometry:", "File already exists:%S\n", "Odometry");
                op.telemetry.update();
            }
            FileWriter wFTCfile = new FileWriter(myFTCfile);*/

            while (op.opModeIsActive() && (difference >= 1)) {
                currentPosition = track();
            /*op.telemetry.addData("targetx", target_position[0]);
            op.telemetry.addData("targety",target_position[1]);
            op.telemetry.addData("angle",angleInRadians);
            op.telemetry.addData("distance",difference);
            op.telemetry.addData("power1",anglePower[0]);
            op.telemetry.addData("power2",anglePower[1]);
            op.telemetry.update();
            op.telemetry.update();*/
                if (difference < 5) {
                    power *= difference / 10;
                    if (abs(power) < 0.2) {
                        power = 0.2;
                    }
                }
                x = target_position[0] - currentPosition[0];
                y = target_position[1] - currentPosition[1];
                angleInRadians = atan2(y, x) - currentPosition[2] * PI / 180;
                anglePower[0] = sin(angleInRadians + PI / 4);
                anglePower[1] = sin(angleInRadians - PI / 4);
                anglecorrection = (currentPosition[2] - target_position[2]) * 0.007;
                /*if (difference > 10) {
                    if (abs(anglePower[1]) > abs(anglePower[0])) {
                        anglePower[1] *= abs(1 / anglePower[1]);
                        anglePower[0] *= abs(1 / anglePower[1]);
                    } else {
                        anglePower[1] *= abs(1 / anglePower[0]);
                        anglePower[0] *= abs(1 / anglePower[0]);

                    }*/
                //}
                motorRightBack.setPower(power * anglePower[1] + anglecorrection);
                motorRightFront.setPower(power * anglePower[0] + anglecorrection);
                motorLeftBack.setPower(power * anglePower[0] - anglecorrection);
                motorLeftFront.setPower(power * anglePower[1] - anglecorrection);
                difference = abs(sqrt((x) * (x) + (y) * (y)));
                op.telemetry.addData("distance", difference);
                op.telemetry.update();
                //op.sleep(3000);
                //FileWriteHandle;
                /*wFTCfile.write(System.currentTimeMillis() + "," + String.format("%.2f",currentPosition[0]) + "," + String.format("%.2f",currentPosition[1]) + "," +
                        String.format("%.2f",currentPosition[2]) + "," +
                        String.format("%.2f",power) + "," +
                        String.format("%.2f",anglePower[0]) + "," +
                        String.format("%.2f",anglePower[1]) + "," +
                        String.format("%.2f",anglecorrection) + "," +
                        String.format("%.2f",difference) +","+"\n"+
                        String.format("%.2f",encoder[0])+"," +String.format("%.2f",encoder[1])+","+ String.format("%.2f",encoder[2])+","+String.format("%.2f",encoder[3])+","+"\n");
            */}
            /*wFTCfile.close();
        } catch (IOException e) {
            e.printStackTrace();
        }*/
        turn(0,0.25);
        stopAllMotors();
    }
}
