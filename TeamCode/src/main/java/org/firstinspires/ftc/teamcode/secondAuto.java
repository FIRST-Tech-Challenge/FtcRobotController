package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="secondAuto", group="Auto")
public class secondAuto extends OpMode {
    static class PIDController {
        private static float p;
        private static float i;
        private static float d;
        private static float integralSummation;
        private static float lastError;
        ElapsedTime timer;
        public void init(float Xp, float Xi, float Xd) {
            PIDController.p = Xp;
            PIDController.i = Xi;
            PIDController.d = Xd;
            timer = new ElapsedTime();
        }
        public float getOutput(float state, float reference) {
            float error = reference - state;
            integralSummation += (float) (error * timer.seconds());
            float derivative = (float) ((error - lastError) / timer.seconds());
            lastError = error;
            return (error * p) + (derivative * d) + (integralSummation * i);
        }
    }

    // Motors & Sensors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor cap;
    private DcMotor spindle;
    private SparkFunOTOS odometry;
    private RevTouchSensor Lswitch;

    // Servos
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;

    // Clock Funcs
    private double lolclock = 0.01;

    // Pose Vars
    private SparkFunOTOS.Pose2D currentPose;
    private SparkFunOTOS.Pose2D previousPose;
    private Long timeAtLastCycle;
    private SparkFunOTOS.Pose2D goal;

    // Position Controllers
    private PIDController translationX;
    private PIDController translationY;
    private PIDController headingPID;

    @Override
    public void init() {
        // Initialize hardware map
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        spindle = hardwareMap.get(DcMotor.class, "spindle");
        odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");
        cap = hardwareMap.get(DcMotor.class, "cap");
        LSLower = hardwareMap.get(ServoImplEx.class, "LSLower");
        LSTop = hardwareMap.get(ServoImplEx.class, "LSTop");
        Lswitch = hardwareMap.get(RevTouchSensor.class, "Lswitch");

        // Define Servo range
        LSLower.setPwmEnable();
        LSTop.setPwmEnable();
        LSLower.scaleRange(0, 1);
        LSTop.scaleRange(0, 1);

        // Set motor directions
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Breaking mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Make sure odo is ready
        odometry.begin();
        odometry.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));
        goal = new SparkFunOTOS.Pose2D(12, 12, 90);

        translationX = new PIDController();
        translationY = new PIDController();
        headingPID = new PIDController();
        currentPose = new SparkFunOTOS.Pose2D();
        previousPose = new SparkFunOTOS.Pose2D();

        translationX.init(0.0f, 0.0f, 0.0f);
        translationY.init(0.0f, 0.0f, 0.0f);
        headingPID.init(0.05f, 0.0f, 0.0f);

        currentPose.set(odometry.getPosition());
        previousPose.set(odometry.getPosition());
        timeAtLastCycle = System.currentTimeMillis();
        telemetry.addData("CurrentPoseX", currentPose.x);
        telemetry.addData("CurrentPoseY", currentPose.y);
        telemetry.addData("CurrentPoseAngle", currentPose.h);
        telemetry.addData("GoalPoseX", currentPose.x);
        telemetry.addData("GoalPoseY", currentPose.y);
        telemetry.addData("GoalPoseAngle", currentPose.h);
        telemetry.update();
    }

    @Override
    public void loop() {
        currentPose.set(odometry.getPosition());
        if (lolclock > 0.05) {
            lolclock = lolclock - 0.001;
        } else {
            lolclock = lolclock + 0.001;
        }

        float x = translationX.getOutput((float) currentPose.x, (float) goal.x);
        float y = translationY.getOutput((float) currentPose.y, (float) goal.y);
        float rx = headingPID.getOutput((float) -currentPose.h, (float) goal.h);
        telemetry.addData("PowX", x);
        telemetry.addData("PowY", y);
        telemetry.addData("PowRX", rx);

        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);

        if ((goal.x-0.2) < currentPose.x && currentPose.x < (goal.x + 0.2) &&
                (goal.y-0.2) < currentPose.y && currentPose.y < (goal.y + 0.2) &&
                (goal.x - 5) < currentPose.h && currentPose.h < (goal.h + 5)) {
            telemetry.addData("R", "STOP");
        }

        previousPose.set(currentPose);
        timeAtLastCycle = System.currentTimeMillis();
        telemetry.addData("CurrentPoseX", currentPose.x);
        telemetry.addData("CurrentPoseY", currentPose.y);
        telemetry.addData("CurrentPoseAngle", currentPose.h);
        telemetry.addData("GoalPoseX", goal.x);
        telemetry.addData("GoalPoseY", goal.y);
        telemetry.addData("GoalPoseAngle", goal.h);
        telemetry.update();
    }
}
