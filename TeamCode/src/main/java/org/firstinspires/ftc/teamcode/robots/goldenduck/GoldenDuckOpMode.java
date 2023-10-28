package org.firstinspires.ftc.teamcode.robots.goldenduck;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.pipeline.AprilTagDetectionPipeline;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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

    @Autonomous(name="Iron Core OpMode", group="Challenge")
    public class PowerPlayIronCore extends OpMode {
        //variable setup
        private DcMotor motorFrontRight = null;
        private DcMotor motorBackLeft = null;
        private DcMotor motorFrontLeft = null;
        private DcMotor motorBackRight = null;
        private DcMotor arm = null;
        private DcMotor elbow = null;
        private Servo claw = null;
        private Servo wrist = null;
        // regular drive
        private double powerLeft = 0;
        private double powerRight = 0;
        // motor power

        private int elbowPosition = 0;
        private int targetElbowPosition = 0;
        private double wristPosition = 0;
        private double targetWristPosition = 0;
        // arm and claw variables
        private int armPosition = 0;
        private int targetArmPos = 0;
        private int maxArm = Integer.MAX_VALUE;
        //number variables
        private static final float DEADZONE = .1f;
        //April Tag stuff
        OpenCvCamera camera;
        AprilTagDetectionPipeline aprilTagDetectionPipeline;

        static final double FEET_PER_METER = 3.28084;

        // Lens intrinsics
        // UNITS ARE PIXELS
        // NOTE: this calibration is for the C920 webcam at 800x448.
        // You will need to do your own calibration for other configurations!
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

        @Override
        public void init() {
            telemetry.addData("Status", "Initializing " + this.getClass()+"...");
            telemetry.addData("Status", "Hold right_trigger to enable debug mode");
            telemetry.update();
            motorFrontLeft = this.hardwareMap.get(DcMotor.class, "motorFrontLeft");
            motorBackLeft = this.hardwareMap.get(DcMotor.class, "motorBackLeft");
            motorFrontRight = this.hardwareMap.get(DcMotor.class, "motorFrontRight");
            motorBackRight = this.hardwareMap.get(DcMotor.class, "motorBackRight");
            arm = this.hardwareMap.get(DcMotor.class, "arm");
            elbow = this.hardwareMap.get(DcMotor.class, "elbow");
            claw = this.hardwareMap.get(Servo.class, "claw");
            wrist = this.hardwareMap.get(Servo.class, "wrist");
            this.motorFrontRight.setDirection(DcMotor.Direction.REVERSE);
            this.motorBackRight.setDirection(DcMotor.Direction.REVERSE);
            arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            resetMotorEncoder();

//        elbow.setTargetPosition(-505);
            wristPosition = wrist.getPosition();
            wrist.setPosition(0);

            //AprilTag stuff
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPSIDE_DOWN);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });
            tagCount=0;
        }
        public void setupArm(boolean init){
            if(init) {
                arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            }
            arm.setTargetPosition(0);
            elbow.setTargetPosition(0);
            arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            arm.setPower(1);
            elbow.setPower(1);
        }

        @Override
        public void init_loop(){
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (gamepad1.a) {
                runAuton = true;
                setupArm(true); //reset encoders
            }
            if (gamepad1.b) {
                runAuton = false;
                setupArm(false);
            }

            if(currentDetections.size() != 0)
            {
                tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == 1 || tag.id ==2 || tag.id == 3)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        tagCount++;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    tagOfInterestCache = tagOfInterest;
                }
                else
                {
                    tagCount = 0;
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                tagCount=0;
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(A tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen a tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }
            telemetry.addLine(String.format("\nRunAuton=%b", runAuton));
            telemetry.update();
        }

        public boolean runAuton = true;
        public AprilTagDetection tagOfInterestCache;
        @Override
        public void loop() {
            updateSensors();
            //tankDrive();
            //process drive inputs
            if (runAuton){
                runAuton = !autonMove(tagOfInterestCache);
            }
            else {
                mecanumDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
            //joystick processing
            presets();
            armMove();
            clawMove();
            updateMotors();
            motorTelemetry();
        }

        public void updateSensors(){
            //get current positions
            armPosition=arm.getCurrentPosition();
            elbowPosition=elbow.getCurrentPosition();
            wristPosition = wrist.getPosition();

        }
        public void updateMotors(){
            arm.setTargetPosition(targetArmPos);
            elbow.setTargetPosition(targetElbowPosition);
            wrist.setPosition(targetWristPosition);
        }

        public void presets() {
            if (gamepad1.a) { //pickup cone
                targetArmPos = -350;
                targetElbowPosition = -591;
                targetWristPosition = 1;
            }
            if (gamepad1.b) { // low junction
                targetArmPos = -771;
                targetElbowPosition = -290;
                targetWristPosition = 0.5;
            }

            if (gamepad1.x) { //medium junction
                targetArmPos = -1440;
                targetElbowPosition = -820;
                targetWristPosition = 0.5;
            }
            if (gamepad1.y) { //high junction
                targetArmPos = -1840;
                targetElbowPosition =  -1050;
                targetWristPosition = 0.85;
            }
        }

        public void tankDrive()
        {
            powerRight = 0;
            powerLeft = 0;

// tanvi is the bestestestestestest

            if(Math.abs(gamepad1.left_stick_y) > DEADZONE)
            {
                powerLeft = gamepad1.left_stick_y;
            }
            if(Math.abs(gamepad1.right_stick_y) > DEADZONE)
            {
                powerRight = gamepad1.right_stick_y;
            }
            motorFrontRight.setPower(powerRight);
            motorFrontLeft.setPower(powerLeft);
            motorBackRight.setPower(powerRight);
            motorBackLeft.setPower(powerLeft);
        }
        public void mecanumDrive(double forward, double strafe, double rotation)
        {
            double r = Math.hypot(strafe, forward);
            double robotAngle = Math.atan2(forward, strafe) - Math.PI / 4;
            double rightX = rotation;
            final double v1 = r * Math.cos(robotAngle) - rightX;
            final double v2 = r * Math.sin(robotAngle) + rightX;
            final double v3 = r * Math.sin(robotAngle) - rightX;
            final double v4 = r * Math.cos(robotAngle) + rightX;
            motorFrontLeft.setPower(v1);
            motorFrontRight.setPower(v2);
            motorBackLeft.setPower(v3);
            motorBackRight.setPower(v4);
        }
        public void armMove()
        {
            telemetry.addData("arm position: ", armPosition);
            telemetry.addData("elbow position: ", elbowPosition);
            telemetry.addData("wrist position: ", wristPosition);
            if(gamepad1.dpad_down) //manually lower arm - shoulder joint
            {
                targetArmPos = armPosition + 30;
                if(targetArmPos > 0 )
                    targetArmPos = 0;
            }
            if(gamepad1.dpad_up) //manually raise arm - shoulder joint
            {
                targetArmPos = armPosition - 30;
                if (targetArmPos < -1890)
                    targetArmPos = -1890;
            }
            if(gamepad1.dpad_right) //contract elbow
            {
                targetElbowPosition = elbowPosition + 30;
                if(targetElbowPosition > 0 )
                    targetElbowPosition = 0;

            }
            if (gamepad1.dpad_left) //extend elbow
            {
                targetElbowPosition = elbowPosition - 30;
                if (targetElbowPosition < -1100) //todo might want to allow a little more manual extension? currently limited to the high junction elbow position
                    targetElbowPosition = -1100;

            }
            if (gamepad1.left_trigger > DEADZONE)
            {
                wrist.setPosition(wrist.getPosition()+.02);
            }
            if (gamepad1.right_trigger > DEADZONE)
            {
                wrist.setPosition(wrist.getPosition()-.02);
            }

        }
        public void clawMove() {
            telemetry.addData("Claw servo position:", claw.getPosition());
            if (gamepad1.left_bumper)
                claw.setPosition(claw.getPosition()+.02);
            if (gamepad1.right_bumper)
                claw.setPosition(claw.getPosition()-.02);
        }
        public void resetMotorEncoder(){
            motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        public double avgMotorEncoder(){
            return (double)((Math.abs(motorBackLeft.getCurrentPosition())+ Math.abs(motorBackRight.getCurrentPosition())+ Math.abs(motorFrontLeft.getCurrentPosition()) + Math.abs(motorFrontRight.getCurrentPosition())))/4.0;
        }

        double encoderAvg = 0.0;
        boolean autonStage1Complete = false;
        public boolean autonMove(AprilTagDetection detection){
            encoderAvg = avgMotorEncoder();
            if (encoderAvg < 1000){
                mecanumDrive(1,0,0);
            }
            else { //we moved forward to next tile
                mecanumDrive(0, 0, 0); //stop robot
                autonStage1Complete = true;
                resetMotorEncoder();
                if(detection.id == 2) return true; //end of auton
            }

            if (autonStage1Complete && detection.id == 1) {
                if (encoderAvg < 1600) {
                    mecanumDrive(0, -1, 0);
                }
                else {
                    mecanumDrive(0, 0, 0);
                    return true;
                }
            }

            if(autonStage1Complete && detection.id == 3) {
                if (encoderAvg < 1600) {
                    mecanumDrive(0, 1, 0);
                }
                else {
                    mecanumDrive(0, 0, 0);
                    return true;
                }
            }

            return false;
        }

        void tagToTelemetry(AprilTagDetection detection)
        {
            telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
            telemetry.addLine(String.format("\nTag Count ID=%d", tagCount));
            telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
            telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
            //telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
            //telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
            //telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
        }
        void motorTelemetry(){
            telemetry.addLine(String.format("\nMotorFrontLeft=%d", motorFrontLeft.getCurrentPosition()));
            telemetry.addLine(String.format("\nMotorFrontRight=%d",motorFrontRight.getCurrentPosition()));
            telemetry.addLine(String.format("\nMotorBackLeft=%d",motorBackLeft.getCurrentPosition()));
            telemetry.addLine(String.format("\nMotorBackRight=%d",motorBackRight.getCurrentPosition()));
            telemetry.addLine(String.format("\nAverageEncoders=%.2f", encoderAvg));

        }

    }

}
