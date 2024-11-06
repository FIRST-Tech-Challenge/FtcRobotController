package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.List;
import java.util.Scanner;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp", group = "Teleops")
public class teleop extends OpMode {

    // Testing
    boolean testingActive = false;
<<<<<<< Updated upstream
    double lastTestTime = 0;
=======
>>>>>>> Stashed changes

    // Field centric stuff
    boolean usingFC = false;
    double initialAngle = 0;
    double prevFCPressTime = 0;

    double curPowerFL = 0;
    double curPowerFR = 0;
    double curPowerBL = 0;
    double curPowerBR = 0;
    //double tarPower = 0;
    double prevTimeFL = 0;
    double prevTimeFR = 0;
    double prevTimeBL = 0;
    double prevTimeBR = 0;


    // File reading stuff
    File file;
    Scanner scan;
    String fileName = "/sdcard/FIRST/PathTest.txt";
    String position;
    int positions = 0;
    String[][] path;
    double dYdX;
    double totalDistance;
    double distance;
    double theta;
    double xCoordinate;
    double yCoordinate;
    double prevReadPressTime = 0;


    // Mechanism stuff
    double CLOSED_OT_POS = 0;
    double OPEN_OT_POS = .5;
    double clawInc = 0;
    double lastClawTime;
    double UP_OT_FLIP_POS = 0;
    double DOWN_OT_FLIP_POS = 1;
    double UP_OT_PIVOT_POS = 1;
    double DOWN_OT_PIVOT_POS = 0;

    double PERP_IT_POS = 0;
    double PAR_IT_POS = .33;
    double CLOSED_IT_POS = 1;
    double OPEN_IT_POS = .76;
    double UP_IT_FLIP_POS = .7;
    double DOWN_IT_FLIP_POS = .42;
    double MID_IT_FLIP_POS = .5;
    double OUT_IT_FLIP_POS = 0;
    double OUT_DOWN_IT_FLIP_POS = .26;


    boolean upPivotOT = true;
    double pivotTimeOT = 0;

    // Total time. Never reset.
    ElapsedTime totalTime = new ElapsedTime();

    public IMU imu;

    // Ease of controls stuff
    RRLocalizationRead localizationRead;


    // Servos and motors for outtake/intake.
    DcMotor outTakeLift;


    Servo outTakeLargePivotExpansion;
    Servo outTakeLargePivotControl;
    Servo outTakeClawPivot;
    Servo outTakeClaw;

    DcMotor inTakeLift;
    Servo inTakeClaw;
    Servo inTakeFlipControl;
    Servo inTakeFlipExpansion;
    Servo inTakeRotator;


    // Motors for drivetrain
    DcMotor frontLeftMotor;
    DcMotor backLeftMotor;
    DcMotor frontRightMotor;
    DcMotor backRightMotor;


    //BNO055IMU.Parameters pars = new BNO055IMU.Parameters();
    //Orientation angles;

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void init(){
        imu = hardwareMap.get(IMU.class, "imu");
        // change it to match the actual orientation of the rev control hub
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        imu.initialize(parameters);
        outTakeLift = hardwareMap.dcMotor.get("otl");
        outTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //outTakeClaw = hardwareMap.servo.get(("otc"));
        //outTakeFlip = hardwareMap.servo.get(("otf"));

        // Intake lift
        inTakeLift = hardwareMap.dcMotor.get("itl");
        inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

       // inTakeFlipControl.getController().pwmEnable();

        // Control hub servos
        inTakeFlipControl = hardwareMap.servo.get(("itfc")); // port 0
        inTakeFlipExpansion = hardwareMap.servo.get("itfe"); // port 1
        inTakeRotator = hardwareMap.servo.get("itr"); // port 2
        inTakeClaw = hardwareMap.servo.get(("itc")); // port 3


        // Expansion hub servos
        outTakeLargePivotExpansion = hardwareMap.servo.get(("otse"));
        outTakeLargePivotControl = hardwareMap.servo.get(("otsc"));
        outTakeClawPivot = hardwareMap.servo.get(("otcp"));
        outTakeClaw = hardwareMap.servo.get(("otc"));

        // Drivetrain initialization
        frontLeftMotor = hardwareMap.dcMotor.get("frontL");
        backLeftMotor = hardwareMap.dcMotor.get("backL");
        frontRightMotor = hardwareMap.dcMotor.get("frontR");
        backRightMotor = hardwareMap.dcMotor.get("backR");

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // field ease stuff
        localizationRead = new RRLocalizationRead();
        localizationRead.initLocalization(hardwareMap);

    }

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void loop() {

        telemetry.addData("Pos: ", localizationRead.returnPose());
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.dpad_up && totalTime.milliseconds() > lastTestTime + 500)
        {
            if (testingActive)
                testingActive = false;
            else
                testingActive = true;
            lastTestTime = totalTime.milliseconds();
        }
        if (testingActive)
            runTesting();
        else {
            //setOutTakeLift();
            setInTakeLift();
            setInTakeClawGrab();
            setInTakeFlip();
            //setOutTakeFlip();
        //    setOutTakeClawGrab();
            setInTakeRotator();


           // readFile();

            runMotors();
            telemetry.update();
        }
    }

    ////////////////////////////////////////////////////
    double acceleratorFR(double tarPower)
    {
        double dt = (totalTime.milliseconds() - prevTimeFR) / 1000.0;
        prevTimeFR = totalTime.milliseconds();
        int multiplier = 0;
        if (tarPower > curPowerFR)
        {
            multiplier = 1;
        }
        else if (tarPower < curPowerFR)
        {
            multiplier = -1;
        }
        else
            multiplier = 0;

        curPowerFR += ((2 * dt) * multiplier);
        telemetry.addData("curPower", curPowerFR);
        telemetry.addData("dt", dt);
        telemetry.addData("mult", multiplier);
        telemetry.addData("tp", tarPower);
        return tarPower;
    }
    ////////////////////////////////////////////////////
    double acceleratorFL(double tarPower)
    {
        double dt = (totalTime.milliseconds() - prevTimeFL) / 1000.0;
        prevTimeFL = totalTime.milliseconds();
        int multiplier = 0;
        if (tarPower > curPowerFL)
        {
            multiplier = 1;
        }
        else if (tarPower < curPowerFL)
        {
            multiplier = -1;
        }
        else
            multiplier = 0;

        curPowerFL += ((2 * dt) * multiplier);
        return tarPower;
    }
    ////////////////////////////////////////////////////
    double acceleratorBR(double tarPower)
    {
        double dt = (totalTime.milliseconds() - prevTimeBR) / 1000.0;
        prevTimeBR = totalTime.milliseconds();
        int multiplier = 0;
        if (tarPower > curPowerBR)
        {
            multiplier = 1;
        }
        else if (tarPower < curPowerBR)
        {
            multiplier = -1;
        }
        else
            multiplier = 0;

        curPowerBR += ((2 * dt) * multiplier);
        return tarPower;
    }
    ////////////////////////////////////////////////////
    double acceleratorBL(double tarPower)
    {
        double dt = (totalTime.milliseconds() - prevTimeBL) / 1000.0;
        prevTimeBL = totalTime.milliseconds();
        int multiplier = 0;
        if (tarPower > curPowerBL)
        {
            multiplier = 1;
        }
        else if (tarPower < curPowerBL)
        {
            multiplier = -1;
        }
        else
            multiplier = 0;

        curPowerBL += ((2 * dt) * multiplier);
        return tarPower;
    }


    ////////////////////////////////////////////////////////////////////////////////
    void runTesting()
    {
        if (gamepad1.a) {
            frontLeftMotor.setPower(.5);
        }
        else
            frontLeftMotor.setPower(0);
        if (gamepad1.b) {
            frontRightMotor.setPower(.5);
        }
        else
            frontRightMotor.setPower(0);
        if (gamepad1.x) {
            backLeftMotor.setPower(.5);
        }
        else
            backLeftMotor.setPower(0);
        if (gamepad1.y) {
            backRightMotor.setPower(.5);
        }
        else
            backRightMotor.setPower(0);

        if (gamepad2.a)
        {
            // up
            inTakeClaw.setPosition(.76);
        }
        if (gamepad2.b)
        {
            //grab
            inTakeFlipExpansion.setPosition(.3);
            //inTakeFlipControl.getController().setServoPosition();
        }
        if (gamepad2.dpad_up)
        {
            //put in transfer
            inTakeFlipExpansion.setPosition(.5);
        }
        if (gamepad2.dpad_down)
        {
            //put in transfer
            inTakeRotator.setPosition(.32);
        }
        if (gamepad2.dpad_left)
        {
            inTakeRotator.setPosition(0);
        }
        if (gamepad2.x)
        {
            inTakeFlipExpansion.setPosition(.7);
        }
        if (gamepad2.y)
        {
            inTakeClaw.setPosition(1);
        }
        if (gamepad2.left_bumper)
        {
            outTakeLift.setPower(.8);
        }
        else if (gamepad2.right_bumper)
        {
            outTakeLift.setPower(-.8);
        }
        else
        {
            outTakeLift.setPower(0);
        }
        if (gamepad2.left_trigger > .1)
        {
            inTakeLift.setPower(.8);
        }
        else if (gamepad2.right_trigger > .1)
        {
            inTakeLift.setPower(-.8);
        }
        else {
            inTakeLift.setPower(0);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    public double returnGyroYaw()
    {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public double getAngle()
    {
        return Math.toDegrees(returnGyroYaw());
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakeLift(){
        outTakeLift.setPower(gamepad2.left_stick_y);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakeClawGrab(){
        if (gamepad2.x && lastClawTime < totalTime.milliseconds() - 500)
        {
            lastClawTime = totalTime.milliseconds();
            clawInc += .1;
            outTakeClaw.setPosition(clawInc);
            outTakeClaw.setDirection(Servo.Direction.REVERSE);
        }
            //outTakeClaw.setPosition(CLOSED_OT_POS);
        if (gamepad2.y && lastClawTime < totalTime.milliseconds() - 500)
        {
            lastClawTime = totalTime.milliseconds();
            clawInc -= .1;
            outTakeClaw.setPosition(clawInc);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakeFlip(){
        if (gamepad2.left_trigger > .1)
            outTakeLargePivotControl.setPosition(DOWN_OT_FLIP_POS);
        if (gamepad2.right_trigger > .1)
            outTakeLargePivotControl.setPosition(UP_OT_FLIP_POS);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setOutTakePivot(){
        if (gamepad2.dpad_right && pivotTimeOT < totalTime.milliseconds() - 500)
        {
            if (upPivotOT)
            {
                outTakeClawPivot.setPosition(DOWN_OT_PIVOT_POS);
                upPivotOT = false;
            }
            else
            {
                outTakeClawPivot.setPosition(UP_OT_PIVOT_POS);
                upPivotOT = true;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setInTakeClawGrab(){
        if (gamepad2.a)
            inTakeClaw.setPosition(CLOSED_IT_POS);
        if (gamepad2.b)
            inTakeClaw.setPosition(OPEN_IT_POS);
        if (gamepad2.dpad_right)
            inTakeClaw.setPosition(.65);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setInTakeFlip(){
        if (gamepad2.dpad_up)
            inTakeFlipExpansion.setPosition(UP_IT_FLIP_POS);
        if (gamepad2.dpad_down)
            inTakeFlipExpansion.setPosition(DOWN_IT_FLIP_POS);
        if (gamepad2.dpad_left)
        {
            inTakeFlipExpansion.setPosition(MID_IT_FLIP_POS);
        }
        if (gamepad2.x)
            inTakeFlipExpansion.setPosition(OUT_IT_FLIP_POS);
        if (gamepad2.y)
            inTakeFlipExpansion.setPosition(OUT_DOWN_IT_FLIP_POS);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setInTakeRotator(){
        if (gamepad2.left_bumper)
            inTakeRotator.setPosition(PAR_IT_POS);
        if (gamepad2.right_bumper)
            inTakeRotator.setPosition(PERP_IT_POS);
    }

    ////////////////////////////////////////////////////////////////////////////////
    public void setInTakeLift(){
        inTakeLift.setPower(gamepad2.right_stick_y);
        if (gamepad2.left_trigger > .1)
        {
            inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (gamepad2.right_trigger > .1)
        {
            inTakeLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    public double getTrueAngle(double heading)
    {
        double angle = returnGyroYaw();
        if (Math.abs(heading - angle) < 180 )
        {
            return heading - angle;
        }
        if (heading - angle >= 180)
        {
            return  360 - heading - angle;
        }
        if (angle - heading <= -180)
        {
            return angle - heading + 360;
        }
        return heading - angle;

    }



    ////////////////////////////////////////////////////////////////////////////////
    void readFile()
    {
        if (gamepad1.y && totalTime.milliseconds() - 500 > prevReadPressTime) {
            prevReadPressTime = totalTime.milliseconds();

            file = new File(fileName);
            try {
                scan = new Scanner(file);
            } catch (FileNotFoundException e) {
                throw new RuntimeException(e);
            }
            while (scan.hasNextLine()) {
                positions++;
                scan.nextLine();
            }
            try {
                scan = new Scanner(file);
            } catch (FileNotFoundException e) {
                throw new RuntimeException(e);
            }
            path = new String[positions][7];
            for (int i = 0; i < path.length; i++) {
                if (scan.hasNextLine()) {
                    position = scan.nextLine();
                    position = position.replaceAll("[^0-9. -]", "");
                    path[i] = position.split(" ");
                }
                try {
                    dYdX = Double.parseDouble(path[i][1]);
                    totalDistance = Double.parseDouble(path[i][2]);
                    distance = Double.parseDouble(path[i][3]);
                    theta = Double.parseDouble(path[i][4]);
                    xCoordinate = Double.parseDouble(path[i][5]);
                    yCoordinate = Double.parseDouble(path[i][6]);
                } catch (NumberFormatException e) {
                    continue;
                }
                telemetry.addLine("dYdX = " + dYdX);
                telemetry.addLine("totalDist = " + totalDistance);
                telemetry.addLine("dist = " + distance);
                telemetry.addLine("theta = " + theta);
                telemetry.addLine("x = " + xCoordinate);
                telemetry.addLine("y = " + yCoordinate);
                telemetry.addLine(" ");
            }
            telemetry.update();
        }
    }

    ////////////////////////////////////////////////////////////////////////////////
    void runMotors()
    {
        // field centric activation
        if (gamepad1.b && totalTime.milliseconds() - 500 > prevFCPressTime) {
            if (!usingFC) {
                initialAngle = getAngle();
                usingFC = true;
            } else {
                usingFC = false;
            }
            prevFCPressTime = totalTime.milliseconds();
        }

        if (usingFC) {
            // this would be start on Xbox (changeable)
            if (gamepad1.options) {
                imu.resetYaw();
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;



            double trueDiff = getTrueAngle(initialAngle);
            double joyAngle = Math.toDegrees(Math.atan2(y, x));
            double trueJoy = 90 - joyAngle;
            if (trueJoy > 180)
            {
                trueJoy = (trueJoy - 360);
            }
            double cos = Math.cos(Math.toRadians(trueJoy - trueDiff));
            double sin = Math.sin(Math.toRadians(trueJoy - trueDiff));

            //double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double multiplier = Math.max(1 - gamepad1.right_trigger, .25);

            double power = Math.min(Math.abs(y) + Math.abs(x), 1);
            frontLeftMotor.setPower(acceleratorFL((power * cos + power * sin + rx) * multiplier));
            backLeftMotor.setPower(acceleratorBL((power * cos - power * sin + rx) * multiplier));
            frontRightMotor.setPower(acceleratorFR((power * cos - power * sin - rx) * multiplier));
            backRightMotor.setPower(acceleratorBR((power * cos + power * sin - rx) * multiplier));
        }
        else {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            double multiplier = Math.max(1 - gamepad1.right_trigger, .25) ;

            frontLeftMotor.setPower(acceleratorFL(frontLeftPower * multiplier));
            backLeftMotor.setPower(acceleratorBL(backLeftPower * multiplier));
            frontRightMotor.setPower(acceleratorFR(frontRightPower * multiplier));
            backRightMotor.setPower(acceleratorBR(backRightPower * multiplier));
        }
    }

    ///////////////////////////////////////////////////////////////////////////////
    private void runVisionMacro() {

        Limelight3A limelight;
        // target class name to detect
        final String TARGET_CLASS_NAME_BLUE = "blue-face";
        final String TARGET_CLASS_NAME_RED = "red-face";
        final String TARGET_CLASS_NAME_YELLOW = "yellow-face";

        // to build a custom rumble sequence.
        Gamepad.RumbleEffect customRumbleEffect;
        customRumbleEffect = new Gamepad.RumbleEffect.Builder()
                // rumble right motor 30% for 100 mSec
                .addStep(0.3, 0.3, 100)
                .build();

        // initialize Limelight and motors
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        frontLeftMotor = hardwareMap.dcMotor.get("frontL");
        backLeftMotor = hardwareMap.dcMotor.get("backL");
        frontRightMotor = hardwareMap.dcMotor.get("frontR");
        backRightMotor = hardwareMap.dcMotor.get("backR");

        // frequency that telemetry is sent to driver hub
        telemetry.setMsTransmissionInterval(11);

        // Switch to neural network pipeline (assuming it's pipeline 1)
        limelight.pipelineSwitch(0);

        // starts looking for data, make sure to call start() or getLatestResult() will return null.

        // initializes the limelight
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        // get Limelight status and update telemetry
        LLStatus status = limelight.getStatus();
        telemetry.addData("LL Temp", "%.1fC", status.getTemp());
        telemetry.addData("LL CPU", "%.1f%%", status.getCpu());
        telemetry.addData("Pipeline", status.getPipelineIndex());
        // get the latest neural network result
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // access classifier results from the neural network
            List<LLResultTypes.ClassifierResult> classifierResults = result.getClassifierResults();

            // check each classifier result for our target object
            boolean targetDetected = false;
            for (LLResultTypes.ClassifierResult cr : classifierResults) {
                telemetry.addData("Class", cr.getClassName());
                telemetry.addData("Confidence", "%.2f", cr.getConfidence());
                // if the target object is detected, stop the robot
                if (cr.getConfidence() > 70) {
                    if (cr.getClassName().equals(TARGET_CLASS_NAME_BLUE) || cr.getClassName().equals(TARGET_CLASS_NAME_YELLOW)) {
                        targetDetected = true;
                        backLeftMotor.setPower(0);
                        frontLeftMotor.setPower(0);
                        frontRightMotor.setPower(0);
                        backRightMotor.setPower(0);
                        telemetry.addData("Status", "Target detected! Robot stopped. Rumblinnn");
                        gamepad1.runRumbleEffect(customRumbleEffect);
                        break;
                    }
                }
            }

            // if the target is not detected, the robot continues to strafe left
            if (!targetDetected) {
                frontLeftMotor.setPower(-0.5);   // Move backward
                frontRightMotor.setPower(0.5);   // Move forward
                backLeftMotor.setPower(0.5);     // Move forward
                backRightMotor.setPower(-0.5);   // Move backward
                telemetry.addData("Status", "Target not detected, strafing left...");
            }

        } else {
            telemetry.addData("Limelight", "No data available");
            frontLeftMotor.setPower(-0.5);   // Move backward
            frontRightMotor.setPower(0.5);   // Move forward
            backLeftMotor.setPower(0.5);     // Move forward
            backRightMotor.setPower(-0.5);   // Move backward
        }

        telemetry.update();
        limelight.stop();
    }

    ////////////////////////////////////////////////////////////////////////////////
    @Override
    public void stop()
    {
        telemetry.addLine("Stopped");
        telemetry.update();
    }
}