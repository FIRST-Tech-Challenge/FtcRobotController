package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="thirdAuto", group="Auto")
public class thirdAuto extends OpMode {
    static class PIDController {
        private static float p;
        private static float i;
        private static float d;
        private static float integralSummation;
        private static float lastError;
        ElapsedTime timer;

        public void init(float Xp, float Xi, float Xd) {
            p = Xp;
            i = Xi;
            d = Xd;
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

    enum armPose {
        BASKET,
        SUBMERSIBLE_A,
        SUBMERSIBLE_B,
        REST,
        CHAMBER_A,
        CHAMBER_B

    }

    enum armPoseZone {
        OTHER,
        CHAMBER,
        SUBMERSABLE
    }

    private static class ArmSubSystem {
        private static MecanumTeleOp.armPose positionalState = MecanumTeleOp.armPose.REST;
        private static MecanumTeleOp.armPoseZone positionalZone = MecanumTeleOp.armPoseZone.OTHER;
        private static DcMotor cap;
        private static DcMotor extendo;
        private static RevTouchSensor ClasslimitSwitch;
        private static MecanumTeleOp.PIDController capstanPID = new MecanumTeleOp.PIDController();
        private static MecanumTeleOp.PIDController extendoPID = new MecanumTeleOp.PIDController();
        private static int capstanReference = 0;
        private static int extendoReference = 0;
        public void init(MecanumTeleOp.armPose positionalStateInitial, DcMotor capstanMotor, DcMotor spindleMotor, RevTouchSensor limitSwitch) {
            positionalState = positionalStateInitial;
            cap = capstanMotor;
            extendo = spindleMotor;
            ClasslimitSwitch = limitSwitch;
            extendoPID.init(0.005F, 0, 0.001F);
            capstanPID.init(0.006F, 0, 0.001F);
        }
        public double getCapstanReference() {
            return capstanReference;
        }
        public double getExtendoReference() {
            return extendoReference;
        }
        public void setReferences(MecanumTeleOp.armPose pose) {
            positionalState = pose;
            switch (positionalState) {
                case SUBMERSIBLE_B:
                    positionalZone = MecanumTeleOp.armPoseZone.SUBMERSABLE;
                    extendoReference = 1700;
                    capstanReference = 117;
                    break;
                case SUBMERSIBLE_A:
                    positionalZone = MecanumTeleOp.armPoseZone.SUBMERSABLE;
                    extendoReference = 1700;
                    capstanReference = 170;
                    break;
                case CHAMBER_B:
                    positionalZone = MecanumTeleOp.armPoseZone.CHAMBER;
                    extendoReference = 0;
                    capstanReference = 400;
                    break;
                case CHAMBER_A:
                    positionalZone = MecanumTeleOp.armPoseZone.CHAMBER;
                    extendoReference = 0;
                    capstanReference = 600;
                    break;
                case BASKET:
                    positionalZone = MecanumTeleOp.armPoseZone.OTHER;
                    extendoReference = 1700;
                    capstanReference = 850;
                    break;
                case REST:
                    positionalZone = MecanumTeleOp.armPoseZone.OTHER;
                    extendoReference = 0;
                    capstanReference = 200;
                    break;
            }
        }
        public void periodicUpdate() {
            extendo.setPower(extendoPID.getOutput(extendo.getCurrentPosition(), extendoReference));
            if (!ClasslimitSwitch.isPressed()) {
                cap.setPower(capstanPID.getOutput(cap.getCurrentPosition(), capstanReference));
            }
        }
        public void cycleSubmersible() {
            if (positionalZone != MecanumTeleOp.armPoseZone.SUBMERSABLE) {
                setReferences(MecanumTeleOp.armPose.SUBMERSIBLE_A);
            } else if (positionalState == MecanumTeleOp.armPose.SUBMERSIBLE_A) {
                setReferences(MecanumTeleOp.armPose.SUBMERSIBLE_B);
            } else {
                setReferences(MecanumTeleOp.armPose.REST);
            }
        }
        public void cycleBasketTop() {
            if (positionalState != MecanumTeleOp.armPose.BASKET) {
                setReferences(MecanumTeleOp.armPose.BASKET);
            } else {
                setReferences(MecanumTeleOp.armPose.REST);
            }
        }
        public void cycleChamberTop() {
            if (positionalZone != MecanumTeleOp.armPoseZone.CHAMBER) {
                setReferences(MecanumTeleOp.armPose.CHAMBER_A);
            } else if (positionalState == MecanumTeleOp.armPose.CHAMBER_A) {
                setReferences(MecanumTeleOp.armPose.CHAMBER_B);
            } else {
                setReferences(MecanumTeleOp.armPose.REST);
            }
        }
        public void goToRest() {
            setReferences(MecanumTeleOp.armPose.REST);
        }
    }

    private static class DriveBaseSubsystem {
        // Motors & Sensors
        private static DcMotor _frontLeftMotor;
        private static DcMotor _frontRightMotor;
        private static DcMotor _backLeftMotor;
        private static DcMotor _backRightMotor;
        private static SparkFunOTOS _odometry;

        // Position Controllers
        private static PIDController translationX;
        private static PIDController translationY;
        private static PIDController headingPID;

        // Pose Vars
        private static SparkFunOTOS.Pose2D currentPose;
        private static SparkFunOTOS.Pose2D previousPose;
        private static Long timeAtLastCycle;
        private static SparkFunOTOS.Pose2D goal;
        private static boolean isBusy = true;

        // Path Planning
        private static double[][] path;
        private static int pathStage = 0;

        // Start Next Path
        private static boolean startNextPath = true;

        public void init(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, SparkFunOTOS odometry, Telemetry telemetry, double[][] _path) {
            // Set Path
            path = _path;

            // Motors and Sensors
            _frontLeftMotor = frontLeftMotor;
            _frontRightMotor = frontRightMotor;
            _backLeftMotor = backLeftMotor;
            _backRightMotor = backRightMotor;
            _odometry = odometry;

            // PID Stuff
            translationX = new PIDController();
            translationY = new PIDController();
            headingPID = new PIDController();
            translationX.init(2F, 0.0F, 0.0F);
            translationY.init(2F, 0.0F, 0.0F);
            headingPID.init(0F, 0.0F, 0.0F);

            // Make sure odo is ready
            odometry.begin();
            odometry.setPosition(new SparkFunOTOS.Pose2D(0.0, 0.0, 0.0));

            // Set Units
            _odometry.setAngularUnit(AngleUnit.DEGREES);
            _odometry.setLinearUnit(DistanceUnit.INCH);
            _odometry.resetTracking();

            goal = new SparkFunOTOS.Pose2D(path[pathStage][0], path[pathStage][1], path[pathStage][2]);

            currentPose = new SparkFunOTOS.Pose2D();
            previousPose = new SparkFunOTOS.Pose2D();

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
            previousPose.set(currentPose);
            timeAtLastCycle = System.currentTimeMillis();
        }

        private void driveMech(double x, double y, double rx) {
            _frontLeftMotor.setPower(y + x + rx);
            _backLeftMotor.setPower(y - x + rx);
            _frontRightMotor.setPower(y - x - rx);
            _backRightMotor.setPower(y + x - rx);
        }

        private void periodicUpdate(Telemetry telemetry) {
            currentPose.set(_odometry.getPosition());
//            driveMech(translationX.getOutput((float) currentPose.x, (float) goal.x), translationY.getOutput((float) currentPose.y, (float) goal.y), headingPID.getOutput((float) -currentPose.h, (float) goal.h));
            driveMech(0.25, 0.25, 0);
//            if ((goal.x-0.2) < currentPose.x && currentPose.x < (goal.x + 0.2) &&
//                    (goal.y-0.2) < currentPose.y && currentPose.y < (goal.y + 0.2) &&
//                    (goal.x - 5) < currentPose.h && currentPose.h < (goal.h + 5)) {
//                telemetry.addData("isBusy", "false");
//                if (isBusy) {
//                    startNextPath = false;
//                }
//                if (startNextPath && (pathStage > (path.length + 1))) {
//                    pathStage = pathStage + 1;
//                    goal.set(new SparkFunOTOS.Pose2D(path[pathStage][0], path[pathStage][1], path[pathStage][2]));
//
//                } else {
//                    driveMech(0,0,0);
//                    telemetry.addData("stopped", "true");
//                }
//                isBusy = false;
//            } else {
//                isBusy = true;
//                telemetry.addData("isBusy", "true");
//                driveMech(translationX.getOutput((float) currentPose.x, (float) goal.x), translationY.getOutput((float) currentPose.y, (float) goal.y), headingPID.getOutput((float) -currentPose.h, (float) goal.h));
//            }
            telemetry.addData("Current Path Stage", pathStage);
            telemetry.addData("Current Pose", "(" + currentPose.x + ", " + currentPose.y + ", " + currentPose.x + ")");
            telemetry.addData("Goal Pose", "(" + goal.x + ", " + goal.y + ", " + goal.x + ")");
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().setTranslation(currentPose.x, currentPose.y);
            packet.fieldOverlay().setRotation(Math.toRadians(currentPose.h));
            telemetry.update();

            previousPose.set(currentPose);
            timeAtLastCycle = System.currentTimeMillis();
        }

        private boolean isBusy() {
            return isBusy;
        }

        private void StartNextPath() {
            if (!isBusy) {
                startNextPath = true;
            }
        }
    }

    // Servos
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;

    // Clock Funcs
    private double lolclock = 0.01;

    // ACS & DBS
    private ArmSubSystem ArmControlSubsystem =  new ArmSubSystem();
    private DriveBaseSubsystem DriveBaseSystem = new DriveBaseSubsystem();

    @Override
    public void init() {
        // Initialize hardware map
        // Motors & Sensors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor spindle = hardwareMap.get(DcMotor.class, "spindle");
        SparkFunOTOS odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");
        DcMotor cap = hardwareMap.get(DcMotor.class, "cap");
        LSLower = hardwareMap.get(ServoImplEx.class, "LSLower");
        LSTop = hardwareMap.get(ServoImplEx.class, "LSTop");
        RevTouchSensor lswitch = hardwareMap.get(RevTouchSensor.class, "Lswitch");

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

        // Prep ArmControlSubsystem
        ArmControlSubsystem.init(MecanumTeleOp.armPose.REST, cap, spindle, lswitch);

        // Generate Path
        double[][] path = {{0, 10, 0}, {0, 5, 0}, {-60, 0, 0}};

        // Prep Drive Base Subsystem
        DriveBaseSystem.init(frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor, odometry, telemetry, path);

    }

    @Override
    public void loop() {
        if (lolclock > 0.05) {
            lolclock = lolclock - 0.001;
        } else {
            lolclock = lolclock + 0.001;
        }
        DriveBaseSystem.periodicUpdate(telemetry);
//        ArmControlSubsystem.periodicUpdate();
        if (!DriveBaseSystem.isBusy()) {
            DriveBaseSystem.StartNextPath();
        }
    }
}
