package org.firstinspires.ftc.teamcode.robots.goldenduck;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.pipeline.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config ("GoldenDuckGameVariables")
@TeleOp(name="Golden Duck OpMode", group="Challenge")
public class GoldenDuckOpMode extends OpMode {
    //autonomous variables
    public boolean auton = true; // controls if auton will run set to true to run with auton
    public static boolean testing = false;// turns off normal robot motion
    public static boolean red = true; // team boolean variable red true is red team
    public static boolean farmCones = false;
    //miscellaneous variables
    public static boolean calibrateOn = true;// turns off automatic elevator calibration
    private boolean calibrate = false;
    public static float DEADZONE = .1f;
    //vision variables
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    static final double FEET_PER_METER = 3.28084;
    int tagDetected = 0;
    // UNITS ARE PIXELS
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.045; //tag size on iron reign signal sleeve
    int ID_TAG_OF_INTEREST = 1; // Tag ID 1 from the 36h11 family
    int tagCount = 0;
    boolean tagFound = false;
    AprilTagDetection tagOfInterest = null;
    //Robot variable storage system
    DriveTrain driveTrain;
    @Override
    public void init() {
        driveTrain = new DriveTrain(telemetry, hardwareMap);
        driveTrain.motorInit();
    }

    @Override
    public void init_loop() {
        telemetry.update();
    }

    @Override
    public void loop() {

        driveTrain.mechanumDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        if (gamepad1.dpad_down) {
            calibrate = false;
        }
        if (gamepad1.dpad_up) {
            if (driveTrain.robotSpeed == 1)
                driveTrain.robotSpeed = .5;
            else
             driveTrain.robotSpeed = 1;
        }
    }
    class DriveTrain {
        private Telemetry telemetry;
        private HardwareMap hardwareMap;
        private DcMotorEx motorFrontRight = null;
        private DcMotorEx motorBackLeft = null;
        private DcMotorEx motorFrontLeft = null;
        private DcMotorEx motorBackRight = null;
        // regular drive
        private double powerLeft = 0;
        private double powerRight = 0;
        // mecanum types
        private double powerFrontLeft = 0;
        private double powerFrontRight = 0;
        private double powerBackLeft = 0;
        private double powerBackRight = 0;
        // Number variables
        private static final float DEADZONE = .1f;
        double robotSpeed = 1;
        public DriveTrain(Telemetry telemetry, HardwareMap hardwareMap)
        {
            this.telemetry = telemetry;
            this.hardwareMap = hardwareMap;
        }
        public void resetMotors() {
            motorFrontLeft.setPower(1);
            motorBackLeft.setPower(1);
            motorBackRight.setPower(1);
            motorFrontRight.setPower(1);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
            this.motorBackLeft.setDirection(DcMotor.Direction.REVERSE);
            motorFrontLeft.setPower(1);
            motorBackLeft.setPower(1);
            motorBackRight.setPower(1);
            motorFrontRight.setPower(1);
            mechanumDrive(0, 0, 0);
        }
        public void tankDrive(double left, double right) {
            powerRight = 0;
            powerLeft = 0;
            if (Math.abs(left) > DEADZONE) {
                powerLeft = left;
            }
            if (Math.abs(right) > DEADZONE) {
                powerRight = right;
            }
            motorFrontRight.setPower(powerRight);
            motorFrontLeft.setPower(powerLeft);
            motorBackRight.setPower(powerRight);
            motorBackLeft.setPower(powerLeft);
        }

        public void mechanumDrive(double forward, double strafe, double turn) {
            forward = -forward;
            turn = -turn;
            double r = Math.hypot(strafe, forward);
            double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
            double rightX = turn;
            powerFrontLeft = r * Math.cos(robotAngle) - rightX;
            powerFrontRight = r * Math.sin(robotAngle) + rightX;
            powerBackLeft = r * Math.sin(robotAngle) - rightX;
            powerBackRight = r * Math.cos(robotAngle) + rightX;
            motorFrontLeft.setPower(powerFrontLeft*robotSpeed);
            motorFrontRight.setPower(powerFrontRight*robotSpeed);
            motorBackLeft.setPower(powerBackLeft*robotSpeed);
            motorBackRight.setPower(powerBackRight*robotSpeed);
        }
        public void telemetryOutput()
        {
            telemetry.addData("Back Right Position \t", motorBackRight.getCurrentPosition());
            telemetry.addData("Back Left Position \t", motorBackLeft.getCurrentPosition());
            telemetry.addData("Front Right Position \t", motorFrontRight.getCurrentPosition());
            telemetry.addData("Front Left Position \t", motorFrontLeft.getCurrentPosition());
        }
        public void motorInit()
        {
            motorFrontLeft = this.hardwareMap.get(DcMotorEx.class, "motorFrontLeft");
            motorBackLeft = this.hardwareMap.get(DcMotorEx.class, "motorBackLeft");
            motorFrontRight = this.hardwareMap.get(DcMotorEx.class, "motorFrontRight");
            motorBackRight = this.hardwareMap.get(DcMotorEx.class, "motorBackRight");
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motorBackLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorBackRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            motorFrontRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
            this.motorBackLeft.setDirection(DcMotorEx.Direction.REVERSE);
            this.motorFrontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        }
        public double getMotorAvgPosition(){return (double)(Math.abs(motorFrontLeft.getCurrentPosition())+Math.abs(motorFrontRight.getCurrentPosition())+Math.abs(motorBackLeft.getCurrentPosition())+Math.abs(motorBackRight.getCurrentPosition()))/4.0;}
    }

}
