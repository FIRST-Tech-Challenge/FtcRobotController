package org.firstinspires.ftc.team417_CENTERSTAGE;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.team417_CENTERSTAGE.opencv.Constants;
import org.firstinspires.ftc.team417_CENTERSTAGE.opencv.OpenCvColorDetection;

@Config
abstract public class BaseAutonomous extends BaseOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    int lastEncoderFL = 0;
    int lastEncoderFR = 0;
    int lastEncoderBL = 0;
    int lastEncoderBR = 0;

    // Autonomous tuning constants
    public static double LEFT_Y = 26.0;
    public static double LEFT_X = -12.0;
    public static double RIGHT_Y = 26.0;
    public static double RIGHT_X = 12.0;
    public static double CENTER_Y = 29.0;
    public static double CENTER_X = 4.5;
    public static double INTAKE_SPEED = 1;
    public static double INTAKE_TIME = 3000; // in milliseconds

    public static double INTAKE_SPEED2 = 1;
    public static double INTAKE_TIME2 = 10000; // in milliseconds
    public static double MOVING_FROM_WALL = 3.0;
    public static double FAR_PARKING = 96;
    public static double CLOSE_PARKING = 48;
    public static double ROBOT_SPEED = 0.5;
    public static double STRAFE_FACTOR = 1.210719915922228;
    public static double DISTANCE_FACTOR = 1.032258064516129;

    public static double Y_CALIBRATION_RIGHT = -2.0;
    public static double Y_CALIBRATION_LEFT = 1.0;


    public void driveInches(double x, double y) {
        double xTicks = x * TICKS_PER_INCH * STRAFE_FACTOR;
        double yTicks = y * TICKS_PER_INCH * DISTANCE_FACTOR;

        double targetFL = xTicks + yTicks;
        double targetFR = yTicks - xTicks;
        double targetBL = yTicks - xTicks;
        double targetBR = yTicks + xTicks;

        // Determine new target position, and pass to motor controller
        targetFL += FL.getCurrentPosition();
        targetFR += FR.getCurrentPosition();
        targetBL += BL.getCurrentPosition();
        targetBR += BR.getCurrentPosition();

        FL.setTargetPosition((int) targetFL);
        FR.setTargetPosition((int) targetFR);
        BL.setTargetPosition((int) targetBL);
        BR.setTargetPosition((int) targetBR);

        // Turn On RUN_TO_POSITION
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        FL.setPower(ROBOT_SPEED);
        FR.setPower(ROBOT_SPEED);
        BL.setPower(ROBOT_SPEED);
        BR.setPower(ROBOT_SPEED);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while (opModeIsActive() &&
                (runtime.seconds() < 30) &&
                (FL.isBusy() && FR.isBusy() && BL.isBusy() && BR.isBusy())) {
        }

        // Stop all motion;
        stopDriving();

        // Turn off RUN_TO_POSITION
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    OpenCvColorDetection myColorDetection = new OpenCvColorDetection(this);

    public void initializeAuto() {
        telemetry.addData("Init State", "Init Started");
        telemetry.update();
        initializeHardware();
        myColorDetection.init();

        telemetry.addData("Init State", "Init Finished");

        // Set last know encoder values
        lastEncoderFR = FR.getCurrentPosition();
        lastEncoderFL = FL.getCurrentPosition();
        lastEncoderBL = BL.getCurrentPosition();
        lastEncoderBR = BR.getCurrentPosition();

        sleep(5000);

        telemetry.clear();
        telemetry.addLine("Initialized. Ready to start!");
        telemetry.update();
    }
    private void stopDriving() {
        FL.setPower(0.0);
        FR.setPower(0.0);
        BL.setPower(0.0);
        BR.setPower(0.0);
    }

    public void runSimpleInchesAuto(boolean red, boolean close) {
        if (red) {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.RED);
            //telemetry.addData("Blue")
        } else {
            myColorDetection.setDetectColor(OpenCvColorDetection.detectColorType.BLUE);
        }

        initializeAuto();

        waitForStart();
        Log.d("skid", toString());

        double x = 0;
        double y = 0;
        switch (myColorDetection.detectTeamProp()) {
            case LEFT:
                telemetry.addData("Side", "Left");
                x = LEFT_X;
                y = LEFT_Y;
                break;
            case CENTER:
                telemetry.addData("Side", "Center");
                x = CENTER_X;
                y = CENTER_Y;
                break;
            case RIGHT:
                telemetry.addData("Side", "Right");
                x = RIGHT_X;
                y = RIGHT_Y;
                break;
            default:
                telemetry.addData("Side", "Unsure");
        }
        telemetry.update();
        driveInches(0, y);
        driveInches(x, 0);

        if(intakeMotor != null) {
              intakeMotor.setPower(INTAKE_SPEED);
              sleep((long) INTAKE_TIME);
              intakeMotor.setPower(0);
        } else {
            sleep(5000); 
        }

        driveInches(-x, 0);
        driveInches(0, -y);
        driveInches(0, MOVING_FROM_WALL);

        if (close) {
            if (red) {
                driveInches(CLOSE_PARKING, Y_CALIBRATION_RIGHT);
            } else {
                driveInches(-CLOSE_PARKING, Y_CALIBRATION_LEFT);
            }
        } else {
            if (red) {
                driveInches(FAR_PARKING, Y_CALIBRATION_RIGHT);
            } else {
                driveInches(-FAR_PARKING, Y_CALIBRATION_LEFT);
            }
        }

        if (intakeMotor != null) {
            intakeMotor.setPower(-INTAKE_SPEED2);
            sleep((long) INTAKE_TIME2);
            intakeMotor.setPower(0);
        } else {
            sleep(5000);
        }
    }

    @Override
    public String toString() {
        return "BaseAutonomous{" +
                "runtime=" + runtime +
                ", lastEncoderFL=" + lastEncoderFL +
                ", lastEncoderFR=" + lastEncoderFR +
                ", lastEncoderBL=" + lastEncoderBL +
                ", lastEncoderBR=" + lastEncoderBR +
                ", LEFT_Y=" + LEFT_Y +
                ", LEFT_X=" + LEFT_X +
                ", RIGHT_Y=" + RIGHT_Y +
                ", RIGHT_X=" + RIGHT_X +
                ", CENTER_Y=" + CENTER_Y +
                ", CENTER_X=" + CENTER_X +
                ", INTAKE_SPEED=" + INTAKE_SPEED +
                ", INTAKE_TIME=" + INTAKE_TIME +
                ", MOVING_FROM_WALL=" + MOVING_FROM_WALL +
                ", FAR_PARKING=" + FAR_PARKING +
                ", CLOSE_PARKING=" + CLOSE_PARKING +
                ", ROBOT_SPEED=" + ROBOT_SPEED +
                ", STRAFE_FACTOR=" + STRAFE_FACTOR +
                ", DISTANCE_FACTOR=" + DISTANCE_FACTOR +
                ", LOWER_BLUE=" + Constants.BLUE_COLOR_DETECT_MIN_HSV +
                ", UPPER_BLUE=" + Constants.BLUE_COLOR_DETECT_MAX_HSV +
                ", LOWER_RED=" + Constants.RED_COLOR_DETECT_MIN_HSV +
                ", UPPER_RED=" + Constants.RED_COLOR_DETECT_MAX_HSV +
                ", detectingBlue=" + myColorDetection.myColor +
                ", sideDetected=" + myColorDetection.sideDetected +
                '}';
    }
}
