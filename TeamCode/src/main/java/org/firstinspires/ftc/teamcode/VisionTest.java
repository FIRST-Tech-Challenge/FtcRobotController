package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot.Hitter;
import org.firstinspires.ftc.robot.WobbleSystem;
import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot_utilities.Vals;
import org.firstinspires.ftc.robot_utilities.VisionController;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name = "VisionTest")
public class VisionTest extends LinearOpMode {

    // Code for vision testing.
    VisionController visionController;
    OpenCvInternalCamera phoneCam;
    ElapsedTime elapsedTime;

    private Motor driveLeft, driveRight;
    private FlyWheel flywheel;
    private Hitter hitter;
    private Motor intake1, intake2;
    private BNO055IMU imu;

    private Orientation lastAngles;
    private double currentHeading = 0;
    private PIDController pidRotate;

    private double driveSpeed = 0.4;
    double rotatePower = 0;

    private GamePadController gamepad;
    private boolean wobbleHandOpen = false;
    private WobbleArmState wobbleArmState = WobbleArmState.UP;
    private WobbleSystem wobbleSystem;

    public void initRobot() {

        elapsedTime = new ElapsedTime();

        //Initialize gamepad.
        gamepad = new GamePadController(gamepad1);

        wobbleSystem = new WobbleSystem(new Motor(hardwareMap, "wobbleArmMotor"),
                hardwareMap.servo.get("wobbleArmServo"));

        driveLeft = new Motor(hardwareMap, "dl");
        driveRight = new Motor(hardwareMap, "dr");
        driveRight.setInverted(true);
        driveLeft.setRunMode(Motor.RunMode.RawPower);
        driveRight.setRunMode(Motor.RunMode.RawPower);

        intake1 = new Motor(hardwareMap, "in1");
        intake2 = new Motor(hardwareMap, "in2");
        intake1.setRunMode(Motor.RunMode.VelocityControl);
        intake2.setRunMode(Motor.RunMode.VelocityControl);
        intake1.setVeloCoefficients(0.05, 0, 0);
        intake2.setVeloCoefficients(0.05, 0, 0);

        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE), telemetry);
        hitter = new Hitter(hardwareMap.servo.get("sv"));

        pidRotate = new PIDController(Vals.rotate_kp, Vals.rotate_ki, Vals.rotate_kd);
        pidRotate.setTolerance(Vals.rotate_tolerance);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        resetAngle();
    }

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        elapsedTime = new ElapsedTime();
        visionController = new VisionController(phoneCam);

        initRobot();
        waitForStart();
        elapsedTime.reset();
        while (elapsedTime.seconds() < 2) {
            telemetry.addData("Position", visionController.getRingPosition());
//            telemetry.addData("Analysis", visionController.getAnalysis());
            telemetry.addData("Height", visionController.getHeight());
            telemetry.update();

            sleep(1000);
        }

        elapsedTime.reset();

        switch (wobbleArmState) {
            case UP:
                wobbleSystem.arm_up();
                break;
            case MID:
                wobbleSystem.arm_mid();
                break;
            case DOWN:
                wobbleSystem.arm_down();
                break;
        }

        if (visionController.getHeight().equals("ZERO")) {

            while (elapsedTime.seconds() < 2) {

                rotatePower = rotate(0);
                double leftPower = -rotatePower;
                double rightPower = rotatePower;

                leftPower += driveSpeed;
                rightPower += driveSpeed;


                driveLeft.set(leftPower);
                driveRight.set(rightPower);

            }

            while (elapsedTime.seconds() < 2.18) {

                driveLeft.set(0);
                driveRight.set(0);

            }

            wobbleArmState = WobbleArmState.DOWN;
            wobbleSystem.hand_open();
            wobbleHandOpen = true;
        }

        if (visionController.getHeight().equals("ONE")) {

            while (elapsedTime.seconds() < 1.5) {

                rotatePower = rotate(0);
                double leftPower = -rotatePower;
                double rightPower = rotatePower;

                leftPower += driveSpeed;
                rightPower += driveSpeed;


                driveLeft.set(leftPower);
                driveRight.set(rightPower);

            }

            while (elapsedTime.seconds() < 1.68) {

                rotatePower = rotate(50);
                driveLeft.set(0);
                driveRight.set(0);

            }

            wobbleArmState = WobbleArmState.DOWN;
            wobbleSystem.hand_open();
        }

        if (visionController.getHeight().equals("FOUR")) {

            while (elapsedTime.seconds() < 1.5) {

                rotatePower = rotate(0);
                double leftPower = -rotatePower;
                double rightPower = rotatePower;

                leftPower += driveSpeed;
                rightPower += driveSpeed;


                driveLeft.set(leftPower);
                driveRight.set(rightPower);

            }

            while (elapsedTime.seconds() < 1.68) {

                rotatePower = rotate(45);
                driveLeft.set(0);
                driveRight.set(0);

            }

            wobbleArmState = WobbleArmState.DOWN;
            wobbleSystem.hand_open();
            wobbleHandOpen = true;
        }
    }

    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = 0;
        pidRotate.reset();
    }

    private double updateHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180) {
            deltaAngle += 360;
        } else if (deltaAngle > 180) {
            deltaAngle -= 360;
        }

        currentHeading += deltaAngle;
        lastAngles = angles;

        return currentHeading;
    }

    private double rotate(double degrees) {
        if (Math.abs(degrees) > 359) degrees = Math.copySign(359, degrees);

        double power = pidRotate.calculate(updateHeading(), degrees);
        return power;

    }


}