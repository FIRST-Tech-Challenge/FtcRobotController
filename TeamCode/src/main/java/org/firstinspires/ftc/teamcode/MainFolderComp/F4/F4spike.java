package org.firstinspires.ftc.teamcode.MainFolderComp.F4;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.MainFolderComp.F4.F4helpers.F4globalVar;
import org.firstinspires.ftc.teamcode.MainFolderComp.F4.F4helpers.F4redPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name="Auton_F4_Spike", group="F4")
// @Disabled
public class F4spike extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    private OpenCvWebcam webcam;
    F4redPipeline pipeline = new F4redPipeline();
    F4globalVar F4var = new F4globalVar();

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: GOBILDA Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_MM   = 96.0 ;     // For figuring circumference
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);

    private DcMotor lfDrive  = null;  //  Used to control the left front drive wheel
    private DcMotor rfDrive   = null;  //  Used to control the right front drive wheel
    private DcMotor lbDrive   = null;  //  Used to control the left back drive wheel
    private DcMotor rbDrive   = null;  //  Used to control the right back drive wheel
    Servo autoarm = null;
    IMU imu = null;

    boolean usingCV = true;

    boolean ready4April = false;

    RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
    RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

    @Override
    public void runOpMode() {


        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam"));

        lfDrive = hardwareMap.get(DcMotor.class, "lf_drive");
        rfDrive = hardwareMap.get(DcMotor.class, "rf_drive");
        lbDrive = hardwareMap.get(DcMotor.class, "lb_drive");
        rbDrive = hardwareMap.get(DcMotor.class, "rb_drive");
        autoarm = hardwareMap.get(Servo.class, "autoy");

        imu = hardwareMap.get(IMU.class, "imu");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        rfDrive.setDirection(DcMotor.Direction.FORWARD);
        lbDrive.setDirection(DcMotor.Direction.REVERSE);
        rbDrive.setDirection(DcMotor.Direction.FORWARD);

        prepareEncoder();


        // Wait for driver to press start
        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                // start streaming the camera
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

            }
            @Override
            public void onError(int errorCode) {

                telemetry.addLine("camera pipeline error");
            }
        });



        // Wait for the game to start (driver presses PLAY) and telemetry for prop location
        while (!opModeIsActive() && !isStarted()) {
            telemetry.addData("Prop Location: ", pipeline.getLocation());
            telemetry.update();
        }
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            if (usingCV) {
                if (getRuntime() >= 3.0) {

                    // Turn team element detection off
                    // pipeline.toggleCVonoff();

                    usingCV = false;
                }
            }

            if (!usingCV && !ready4April) {
                lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                imu.resetYaw();
                moveToProp();

                lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ready4April = true;
                sleep(10);
            }

        }
    }


    public void moveToProp() {

        switch (pipeline.getLocation()) {
            case "middle":

                encoderDriveForwardInches(F4var.F4mSpike1);
                imuCorrection(0, 0.5);

                encoderDriveBackwardInches(F4var.F4mSpike2);
                imuCorrection(0, 0.5);

                lfDrive.setPower(0);
                rfDrive.setPower(0);
                lbDrive.setPower(0);
                rbDrive.setPower(0);



                break;
            case "right":

                encoderDriveRightInches(F4var.F4rSpike1);
                imuCorrection(0, 0.1);

                encoderDriveForwardInches(F4var.F4rSpike2);
                imuCorrection(0, 0.1);

                encoderDriveBackwardInches(F4var.F4rSpike3);
                imuCorrection(0, 0.1);

                lfDrive.setPower(0);
                rfDrive.setPower(0);
                lbDrive.setPower(0);
                rbDrive.setPower(0);

                break;
            case "left":

                encoderDriveForwardInches(F4var.F4lSpike1);
                imuCorrection(0, 0.1);

                TurnLeft(F4var.F4lSpike2);
                imuCorrection(90, 0.1);

                encoderDriveForwardInches(F4var.F4lSpike3);
                imuCorrection(90, 0.1);

                encoderDriveBackwardInches(F4var.F4lSpike4);
                imuCorrection(90, 0.1);

                lfDrive.setPower(0);
                rfDrive.setPower(0);
                lbDrive.setPower(0);
                rbDrive.setPower(0);

                break;
        }

    }

    public void imuCorrection(double directionDeg, double margin) {

        lfDrive.setPower(0);
        lbDrive.setPower(0);
        rfDrive.setPower(0);
        rbDrive.setPower(0);

        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        while (!(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (directionDeg - margin) && imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (directionDeg + margin))) {

            telemetry.addData("Yaw: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < (directionDeg - margin)) {
                lfDrive.setPower(-0.1);
                lbDrive.setPower(-0.1);
                rfDrive.setPower(0.1);
                rbDrive.setPower(0.1);
            } else if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > (directionDeg + margin)) {
                lfDrive.setPower(0.1);
                lbDrive.setPower(0.1);
                rfDrive.setPower(-0.1);
                rbDrive.setPower(-0.1);
            } else {
                lfDrive.setPower(0);
                lbDrive.setPower(0);
                rfDrive.setPower(0);
                rbDrive.setPower(0);
            }
        }

        lfDrive.setPower(0);
        lbDrive.setPower(0);
        rfDrive.setPower(0);
        rbDrive.setPower(0);

        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void prepareEncoder() {

        //  arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void encoderDriveForwardInches(double inches) {
        double TotalTicks = inches*COUNTS_PER_MM*25.4;
        lfDrive.setTargetPosition((int)TotalTicks);
        lbDrive.setTargetPosition((int)TotalTicks);
        rfDrive.setTargetPosition((int)TotalTicks);
        rbDrive.setTargetPosition((int)TotalTicks);
        lfDrive.setPower(0.75);
        lbDrive.setPower(0.75);
        rfDrive.setPower(0.75);
        rbDrive.setPower(0.75);
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*25.4*2));
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void encoderDriveBackwardInches(double Inches) {
        double TotalTicks = Inches*COUNTS_PER_MM*25.4;
        lfDrive.setTargetPosition(-((int)TotalTicks));
        lbDrive.setTargetPosition(-((int)TotalTicks));
        rfDrive.setTargetPosition(-((int)TotalTicks));
        rbDrive.setTargetPosition(-((int)TotalTicks));
        lfDrive.setPower(0.5);
        lbDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        rbDrive.setPower(0.5);
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(Inches*2*25.4));
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveRightInches(double inches) {
        double TotalTicks = inches*COUNTS_PER_MM*25.4;
        lfDrive.setTargetPosition(((int)TotalTicks));
        lbDrive.setTargetPosition(-((int)TotalTicks));
        rfDrive.setTargetPosition(-((int)TotalTicks));
        rbDrive.setTargetPosition(((int)TotalTicks));
        lfDrive.setPower(1);
        lbDrive.setPower(1);
        rfDrive.setPower(1);
        rbDrive.setPower(1);
        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*2*25.4));
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveLeftInches(double inches) {
        double TotalTicks = inches*COUNTS_PER_MM*25.4;
        lfDrive.setTargetPosition(-((int)TotalTicks));
        lbDrive.setTargetPosition(((int)TotalTicks));
        rfDrive.setTargetPosition(((int)TotalTicks));
        rbDrive.setTargetPosition(-((int)TotalTicks));
        lfDrive.setPower(1);
        lbDrive.setPower(1);
        rfDrive.setPower(1);
        rbDrive.setPower(1);

        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*2*25.4));
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void TurnLeft(double degrees) {
        double TotalTicks = (degrees/90)*COUNTS_PER_MOTOR_REV*4.2333333333;
        lfDrive.setTargetPosition(-((int)TotalTicks));
        lbDrive.setTargetPosition(-((int)TotalTicks));
        rfDrive.setTargetPosition(((int)TotalTicks));
        rbDrive.setTargetPosition(((int)TotalTicks));
        lfDrive.setPower(0.5);
        lbDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        rbDrive.setPower(0.5);

        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(degrees/90*960));
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void TurnRight(double degrees) {
        double TotalTicks = (degrees/90)*COUNTS_PER_MOTOR_REV*4.2333333333;
        lfDrive.setTargetPosition(((int)TotalTicks));
        lbDrive.setTargetPosition(((int)TotalTicks));
        rfDrive.setTargetPosition(-((int)TotalTicks));
        rbDrive.setTargetPosition(-((int)TotalTicks));
        lfDrive.setPower(0.5);
        lbDrive.setPower(0.5);
        rfDrive.setPower(0.5);
        rbDrive.setPower(0.5);

        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(degrees/90*960));
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderConcentricRightStrafe(float turnFactor, double inches) {
        double TotalTicks = inches*COUNTS_PER_MM*25.4;
        lfDrive.setTargetPosition(-((int)TotalTicks));
        lbDrive.setTargetPosition(-((int)TotalTicks));
        rfDrive.setTargetPosition(-((int)TotalTicks));
        rbDrive.setTargetPosition(((int)TotalTicks));
        lfDrive.setPower(0.75 - (turnFactor / 10));
        lbDrive.setPower(0.75 - (turnFactor / 10));
        rfDrive.setPower(0.75 + (turnFactor / 10));
        rbDrive.setPower(0.75 + (turnFactor / 10));

        lfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rfDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rbDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(inches*2*25.4));
        lfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}



