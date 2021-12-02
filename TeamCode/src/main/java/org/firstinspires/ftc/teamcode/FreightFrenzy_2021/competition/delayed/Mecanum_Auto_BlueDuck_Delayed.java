package org.firstinspires.ftc.teamcode.FreightFrenzy_2021.competition.delayed;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot_common.Robot4100Common;

import java.util.List;

import static java.lang.Math.toRadians;

@Autonomous(name = "BLUE DUCK DELAYED", group = "Competition")
@Disabled
public class Mecanum_Auto_BlueDuck_Delayed extends LinearOpMode {

    private DcMotor LF = null;
    private DcMotor RF = null;
    private DcMotor LB = null;
    private DcMotor RB = null;
    private DcMotor Intake = null;
    private DcMotor Spin = null;
    private DcMotor Slide = null;
    private Servo Rotate = null;
    private Servo Push = null;
    private ElapsedTime runtime = new ElapsedTime();

    BNO055IMU imu;
    //Vuforia setup for vision
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };
    private static final String VUFORIA_KEY =
            Robot4100Common.VUFORIA_LICENSE;
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        LF = hardwareMap.get(DcMotor.class, "LF");
        RF = hardwareMap.get(DcMotor.class, "RF");
        LB = hardwareMap.get(DcMotor.class, "LB");
        RB = hardwareMap.get(DcMotor.class, "RB");
        Slide = hardwareMap.get(DcMotor.class, "Slide");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Spin = hardwareMap.get(DcMotor.class, "Spin");

        LF.setDirection(DcMotor.Direction.REVERSE);
        RF.setDirection(DcMotor.Direction.FORWARD);
        LB.setDirection(DcMotor.Direction.REVERSE);
        RB.setDirection(DcMotor.Direction.FORWARD);

        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        Intake.setDirection(DcMotor.Direction.REVERSE);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Slide.setDirection(DcMotor.Direction.FORWARD);
        Slide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Spin.setDirection(DcMotor.Direction.FORWARD);
        Spin.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        Rotate = hardwareMap.get(Servo.class, "Rotate");
        Rotate.setDirection(Servo.Direction.FORWARD);
        Push = hardwareMap.get(Servo.class, "Push");

        LF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RF.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initialize position
        Push.setPosition(0.4);
        Slide.setPower(0.15);
        sleep(100);
        Slide.setPower(0.0);
        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rotate.setPosition(0.03);

        //Vision
        initVuforia();
        initTfod();
        if (tfod != null) {
            tfod.activate();
            tfod.setZoom(1, 16.0/9.0);
        }

        //Variables
        double center = -1;
        int initialHeight = Slide.getCurrentPosition();
        String visionResult = null;
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();

        //Traj
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-41, 62.125, toRadians(90));
        drive.setPoseEstimate(startPose);
        Trajectory myTrajectory1 = drive.trajectoryBuilder(startPose,true)
                .splineToConstantHeading(new Vector2d(-11.875, 43), toRadians(-90))
                .build();

        waitForStart();
        if(opModeIsActive()) {

            ElapsedTime recogTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            runtime.reset();
            if (visionResult == null) {
                recogTime.reset();
            }

            while (recogTime.milliseconds() <= 10000.0) {
                if (tfod != null) {
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            if (recognition.getLabel().equals("Duck")) {
                                center = (recognition.getLeft() + recognition.getRight()) / 2.0;
                            }
                            i++;
                        }
                        telemetry.update();
                    }
                }
            }
            if (center < 0) {
                visionResult = "LEFT";
            } else if (center < 321.85) {
                visionResult = "MIDDLE";
            } else {
                visionResult = "RIGHT";
            }
            telemetry.addLine(visionResult);
            telemetry.update();

            //MOTION TO PLATE
            drive.followTrajectory(myTrajectory1);
            sleep(500);

            //ROTATE
            Rotate.setPosition(1.0);
            sleep(800);

            //SLIDE UP
            if (visionResult == "LEFT") {
                Slide.setTargetPosition(initialHeight);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            } else if (visionResult == "MIDDLE") {
                Slide.setTargetPosition(initialHeight + 700);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            } else if (visionResult == "RIGHT") {
                Slide.setTargetPosition(initialHeight + 1500);
                Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Slide.setPower(0.8);
            }
            sleep(600);

            //CLOSER TO PLATE
            Trajectory myTrajectory2 = drive.trajectoryBuilder(myTrajectory1.end())
                    .back(2)
                    .build();
            drive.followTrajectory(myTrajectory2);

            //DUMP AND SLIDE DOWN
            Push.setPosition(0.0);
            sleep(300);
            Push.setPosition(0.4);
            sleep(500);
            Rotate.setPosition(0.03);
            sleep(500);
            Slide.setTargetPosition(initialHeight);
            Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Slide.setPower(0.8);

            //BACK TO WALL
            Trajectory myTrajectory3 = drive.trajectoryBuilder(myTrajectory2.end())
                    .splineToConstantHeading(new Vector2d(-65, 53), toRadians(90))
                    .build();
            drive.followTrajectory(myTrajectory3);
            sleep(500);

            Pose2d wall = new Pose2d(-63.88, 53.25, Math.toRadians(90));
            drive.setPoseEstimate(wall);

            //ROTATE DUCK
            Spin.setPower(0.5);
            sleep(3500);
            Spin.setPower(0.0);

            //PARK
            Trajectory myTrajectory4 = drive.trajectoryBuilder(wall,true)
                    .forward(-18.5)
                    .build();

            drive.followTrajectory(myTrajectory4);

        }

    }

    void twoPhaseSpin(boolean isReversed,double startingSpeed, double endSpeed) {
        double reverseFactor = 1;
        if(isReversed){
            reverseFactor = -1;
        }
        ElapsedTime tSpin = new ElapsedTime();
        double spinPower = startingSpeed * reverseFactor;
        while (tSpin.milliseconds() < 1000){
            Spin.setPower(spinPower);
        }
        while (tSpin.milliseconds() < 2500){
            spinPower = Range.clip(spinPower * tSpin.milliseconds()/1000.0, startingSpeed, endSpeed);
            Spin.setPower(spinPower);
        }
    }

    void stopMotion() {
        LF.setPower(0);
        RF.setPower(0);
        LB.setPower(0);
        RB.setPower(0);
    }

    void stopMotion(long timeInterval) {
        stopMotion();
        sleep(timeInterval);
    }

    double normalizeAngle(double angle) {
        double tempDeg = angle % 360;
        if (tempDeg >= 180) {
            tempDeg -= 360;
        } else if (tempDeg < -180) {
            tempDeg += 360;
        }
        return tempDeg;
    }

    double aquireHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        double tempHead = normalizeAngle(heading);
        telemetry.addData("Heading", tempHead);
        telemetry.update();
        sleep(20);
        return tempHead;
    }

    void displayEncoderValue() {
        try {
            double LFDis = LF.getCurrentPosition() / 537.7 * 1.89 * 2 * 3.1415926;
            double RFDis = RF.getCurrentPosition() / 537.7 * 1.89 * 2 * 3.1415926;
            double LBDis = LB.getCurrentPosition() / 537.7 * 1.89 * 2 * 3.1415926;
            double RBDis = RB.getCurrentPosition() / 537.7 * 1.89 * 2 * 3.1415926;
            telemetry.addData("LF Encoder Value: ", LFDis);
            telemetry.addData("RF Encoder Value: ", RFDis);
            telemetry.addData("LB Encoder Value: ", LBDis);
            telemetry.addData("RB Encoder Value: ", RBDis);
            telemetry.addData("X Value: ", (LFDis / Math.sqrt(2.0) + RBDis / Math.sqrt(2.0) - LBDis / Math.sqrt(2.0) - RFDis / Math.sqrt(2.0)));
            telemetry.addData("Y Value: ", (LFDis / Math.sqrt(2.0) + RFDis / Math.sqrt(2.0) + LBDis / Math.sqrt(2.0) + RBDis / Math.sqrt(2.0)));
            telemetry.addData("Average Encoder Value: ", (LFDis + LBDis + RFDis + RBDis) / 4.0);
            telemetry.update();
        } catch (Exception e) {
            telemetry.addLine("Unable to find encoder value");
            telemetry.update();
        }
    }

    void driveStraight(boolean isForward, double margin, double power, double timeInterval) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double targetAngle = currentAngle;
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;
            if (tempAngle < normalizeAngle(targetAngle - 1 * margin)) {
                RF_power += 0.1;
                RB_power += 0.1;
                LF_power -= 0.1;
                LB_power -= 0.1;
            } else if (tempAngle > normalizeAngle(targetAngle + (margin))) {
                RF_power -= 0.1;
                RB_power -= 0.1;
                LF_power += 0.1;
                LB_power += 0.1;
            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
            displayEncoderValue();
        }
        stopMotion();
    }



    void drivePerpendicularly(boolean isLeft, double margin, double power, double timeInterval) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int perpendicularFactor = -1;
        if (isLeft) {
            perpendicularFactor = 1;
        }
        double targetAngle = normalizeAngle(currentAngle + 90 * perpendicularFactor);
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = -1 * perpendicularFactor * power;
            LB_power = perpendicularFactor * power;
            RF_power = perpendicularFactor * power;
            RB_power = -1 * perpendicularFactor * power;
            if (tempAngle < normalizeAngle(targetAngle - 1 * margin)) {
                RF_power += perpendicularFactor * 0.1;
                RB_power -= perpendicularFactor * 0.1;
                LF_power += perpendicularFactor * 0.1;
                LB_power -= perpendicularFactor * 0.1;
            } else if (tempAngle > normalizeAngle(targetAngle + (margin))) {
                RF_power -= perpendicularFactor * 0.1;
                RB_power += perpendicularFactor * 0.1;
                LF_power -= perpendicularFactor * 0.1;
                LB_power += perpendicularFactor * 0.1;
            }
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
        stopMotion();
    }

    void driveLeftTurn(boolean isForward, double margin, double power, double timeInterval) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {
            double tempAngle = aquireHeading();
            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(-RF_power);
            LB.setPower(LB_power);
            RB.setPower(-RB_power);
            telemetry.addData("RF_power", -RF_power);
            telemetry.addData("RB_power", -RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);

            telemetry.update();
            displayEncoderValue();
        }
        stopMotion();
    }

    void driveRightTurn(boolean isForward, double margin, double power, double timeInterval) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double targetAngle = currentAngle;
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {

            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;

            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(-LF_power);
            RF.setPower(RF_power);
            LB.setPower(-LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", -LF_power);
            telemetry.addData("LB_power", -LB_power);

            telemetry.update();
            displayEncoderValue();
        }
        stopMotion();
    }
    void driveStrafeLeft(boolean isForward, double margin, double power, double timeInterval) {
        ElapsedTime driveTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        final double currentAngle = aquireHeading();
        int straightFactor = -1;
        if (isForward) {
            straightFactor = 1;
        }
        double targetAngle = currentAngle;
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        while (driveTime.milliseconds() < timeInterval) {

            LF_power = straightFactor * power;
            LB_power = straightFactor * power;
            RF_power = straightFactor * power;
            RB_power = straightFactor * power;

            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(0);
            RF.setPower(RF_power);
            LB.setPower(-LB_power);
            RB.setPower(0);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", -LF_power);
            telemetry.addData("LB_power", -LB_power);

            telemetry.update();
            displayEncoderValue();
        }
        stopMotion();
    }

    //make a turn that based on the current heading in a certain direction and angle
    void rotateAtAngle(boolean isClockwise, double degree, double margin, double power) {
        int angleFactor = -1;
        if (!isClockwise) {
            angleFactor = 1;
        }
        final double currentAngle = aquireHeading();
        double targetAngle = normalizeAngle(currentAngle + degree * angleFactor);
        rotateToAngle(targetAngle, margin, power);
        stopMotion();
    }

    //make a turn TO a certain angle
    void rotateToAngle(double targetAngle, double margin, double power) {
        int angleFactor = 0;
        final double currentAngle = aquireHeading();
        if (currentAngle - targetAngle > 0) {
            if (currentAngle - targetAngle < 180) {
                //cw
                angleFactor = -1;
            } else {
                //ccw
                angleFactor = 1;
            }
        } else {
            if (targetAngle - currentAngle < 180) {
                //ccw
                angleFactor = 1;
            } else {
                //cw
                angleFactor = -1;
            }
        }
        double LF_power;
        double LB_power;
        double RF_power;
        double RB_power;
        double tempAngle = currentAngle;
        while (!((tempAngle < targetAngle + margin) && (tempAngle > targetAngle - margin))) {
            tempAngle = aquireHeading();
            RF_power = angleFactor * power;
            RB_power = angleFactor * power;
            LF_power = -1 * angleFactor * power;
            LB_power = -1 * angleFactor * power;
            RF_power = Range.clip(RF_power, -1, 1);
            RB_power = Range.clip(RB_power, -1, 1);
            LF_power = Range.clip(LF_power, -1, 1);
            LB_power = Range.clip(LB_power, -1, 1);
            LF.setPower(LF_power);
            RF.setPower(RF_power);
            LB.setPower(LB_power);
            RB.setPower(RB_power);
            telemetry.addData("RF_power", RF_power);
            telemetry.addData("RB_power", RB_power);
            telemetry.addData("LF_power", LF_power);
            telemetry.addData("LB_power", LB_power);
            telemetry.update();
        }
        stopMotion();
    }

    boolean slideMotion(int targetPosition, double power) {
        Slide.setTargetPosition(targetPosition);
        Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Slide.setPower(power);
        telemetry.addLine("Slide Current: " + Slide.getCurrentPosition());
        telemetry.addLine("Slide Target: " + Slide.getTargetPosition());
        telemetry.update();
        return Slide.isBusy();
    }

    boolean rotateMotion(boolean moveOut) {
        if(moveOut)
            Rotate.setPosition(1.0);
        else
            Rotate.setPosition(0.03);
        telemetry.addLine("Rotate: " + Rotate.getPosition());
        telemetry.update();
        sleep(300);
        return (Rotate.getPosition() == 1.0 || Rotate.getPosition() == 0.03);
    }

    boolean pushMotion(boolean moveOut) {
        if(moveOut)
            Push.setPosition(0.4);
        else
            Push.setPosition(0.0);
        telemetry.addLine("Push: " + Push.getPosition());
        telemetry.update();
        sleep(300);
        return (Push.getPosition() == 0.0 || Push.getPosition() == 0.4);
    }



    boolean spinMotion(double power, double timeInterval) {
        ElapsedTime spinTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        while (spinTime.milliseconds() <= timeInterval) {
            Spin.setPower(power);
        }
        Spin.setPower(0);
        return Spin.isBusy();
    }

    //Vision
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.60f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}