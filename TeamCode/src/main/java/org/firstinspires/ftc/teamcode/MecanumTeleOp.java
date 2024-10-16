package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name="Mecanum TeleOp", group="TeleOp")
public class MecanumTeleOp extends OpMode {

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
        CHAMBER_B,
        ZERO

    }

    enum armPoseZone {
        OTHER,
        CHAMBER,
        SUBMERSABLE
    }

    private static class ArmSubSystem {
        private static armPose positionalState = armPose.REST;
        private static armPoseZone positionalZone = armPoseZone.OTHER;
        private static DcMotor cap;
        private static DcMotor extendo;
        private static RevTouchSensor ClasslimitSwitch;
        private static PIDController capstanPID = new PIDController();
        private static PIDController extendoPID = new PIDController();
        private static int capstanReference = 0;
        private static int extendoReference = 0;
        public void init(armPose positionalStateInitial, DcMotor capstanMotor, DcMotor spindleMotor, RevTouchSensor limitSwitch) {
            positionalState = positionalStateInitial;
            cap = capstanMotor;
            extendo = spindleMotor;
            ClasslimitSwitch = limitSwitch;
            extendoPID.init(0.005F, 0, 0.001F);
            capstanPID.init(0.002F, 0, 0.006F);
        }
        public double getCapstanReference() {
            return capstanReference;
        }
        public double getExtendoReference() {
            return extendoReference;
        }
        public void setReferences(armPose pose) {
            positionalState = pose;
            switch (positionalState) {
                case ZERO:
                    positionalZone = armPoseZone.OTHER;
                    extendoReference = 0;
                    capstanReference = 0;
                    break;
                case SUBMERSIBLE_B:
                    positionalZone = armPoseZone.SUBMERSABLE;
                    extendoReference = 1000;
                    capstanReference = 0;
                    break;
                case SUBMERSIBLE_A:
                    positionalZone = armPoseZone.SUBMERSABLE;
                    extendoReference = 1000;
                    capstanReference = 170;
                    break;
                case CHAMBER_B:
                    positionalZone = armPoseZone.CHAMBER;
                    extendoReference = 0;
                    capstanReference = 370;
                    break;
                case CHAMBER_A:
                    positionalZone = armPoseZone.CHAMBER;
                    extendoReference = 0;
                    capstanReference = 450;
                    break;
                case BASKET:
                    positionalZone = armPoseZone.OTHER;
                    extendoReference = 1000;
                    capstanReference = 850;
                    break;
                case REST:
                    positionalZone = armPoseZone.OTHER;
                    extendoReference = 0;
                    capstanReference = 200;
                    break;
            }
        }
        public void periodicUpdate(Telemetry telemetry) {
            double extendoOutput = extendoPID.getOutput(extendo.getCurrentPosition(), extendoReference);
            if (-0.25 > extendoOutput || extendoOutput > 0.25) {
                extendo.setPower(extendoOutput);
            } else {
                extendo.setPower(0);
            }
            if (!ClasslimitSwitch.isPressed()) {
                cap.setPower(capstanPID.getOutput(cap.getCurrentPosition(), capstanReference));
            }
            telemetry.addData("Extendo Power", extendo.getPower());
        }
        public void cycleSubmersible() {
            if (positionalState == armPose.SUBMERSIBLE_A) {
                setReferences(armPose.SUBMERSIBLE_B);
            } else {
                setReferences(armPose.SUBMERSIBLE_A);
            }
        }
        public void enterZeroPos() {
            setReferences(armPose.ZERO);
        }
        public void cycleBasketTop() {
            if (positionalState != armPose.BASKET) {
                setReferences(armPose.BASKET);
            } else {
                setReferences(armPose.REST);
            }
        }
        public void cycleChamberTop() {
            if (positionalZone != armPoseZone.CHAMBER) {
                setReferences(armPose.CHAMBER_A);
            } else if (positionalState == armPose.CHAMBER_A) {
                setReferences(armPose.CHAMBER_B);
            } else {
                setReferences(armPose.REST);
            }
        }
        public void goToRest() {
            setReferences(armPose.REST);
        }
    }

    // Motors & Sensors
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private SparkFunOTOS odometry;

    // Gamepad2 Toggles
    private boolean gamepad2YClicking = false;
    private boolean gamepad2BClicking = false;
    private boolean gamepad2XClicking = false;
    private boolean gamepad2AClicking = false;

    // Servo Vars
    private double wristReference = 0.5;
    private double gripperReference = 0.5;
    private double lolclock = 0.01;

    // Funzies
    private static final boolean allowAnthonyExtras = false;

    // Servos
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;

    // Subsystem
    private static ArmSubSystem armSystem = new ArmSubSystem();

    @Override
    public void init() {
        // Initialize hardware map
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");

        DcMotor spindle = hardwareMap.get(DcMotor.class, "spindle");
        odometry = hardwareMap.get(SparkFunOTOS.class, "odometry");
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
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set motor modes
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        spindle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Breaking mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Make sure odo is ready
        odometry.begin();

        // Initialize the ArmSubSystem
        armSystem.init(armPose.REST, cap, spindle, lswitch);

    }



    @Override
    public void loop() {
        if (lolclock > 0.05) {
            lolclock = lolclock - 0.001;
        } else {
            lolclock = lolclock + 0.001;
        }
        // Get joystick values
        double y;
        double x;
        double rx;

        if (gamepad1.left_bumper) {
            y = (gamepad1.left_stick_y * 0.25) + (Math.pow(gamepad1.left_stick_y, 3) * 0.2);
            x = (-gamepad1.left_stick_x * 0.25) + (Math.pow(-gamepad1.left_stick_x, 3) * 0.2);
            rx = -gamepad1.right_stick_x * 0.25;
        } else {
            y = Math.pow(gamepad1.left_stick_y, 3) * (1 - gamepad1.left_trigger);
            x = Math.pow(-gamepad1.left_stick_x, 3) * (1 - gamepad1.left_trigger);
            rx = -gamepad1.right_stick_x * (1 - gamepad1.left_trigger);
        }

        // Update References with ArmSystem

        if (gamepad2.a && !gamepad2AClicking) {
            gamepad2AClicking = true;
            armSystem.goToRest(); // If A Clicked Go to Rest
        }
        if (!gamepad2.a) {
            gamepad2AClicking = false;
        }

        if (gamepad2.b && !gamepad2BClicking) {
            gamepad2AClicking = true;
            armSystem.cycleBasketTop(); // If B Clicked Go to Basket
        }
        if (!gamepad2.b) {
            gamepad2BClicking = false;
        }

        if (gamepad2.x && !gamepad2XClicking) {
            gamepad2XClicking = true;
            armSystem.cycleSubmersible(); // If X Clicked Go to Submersible
        }
        if (!gamepad2.x) {
            gamepad2XClicking = false;
        }

        if (gamepad2.y && !gamepad2YClicking) {
            gamepad2YClicking = true;
            armSystem.cycleChamberTop(); // If Y Clicked Go to Top Chamber
        }
        if (!gamepad2.y) {
            gamepad2YClicking = false;
        }
        if (allowAnthonyExtras) {
            if (gamepad1.a && !gamepad2AClicking) {
                gamepad2AClicking = true;
                armSystem.goToRest(); // If A Clicked Go to Rest
            }
            if (!gamepad1.a) {
                gamepad2AClicking = false;
            }

            if (gamepad1.b && !gamepad2BClicking) {
                gamepad2AClicking = true;
                armSystem.cycleBasketTop(); // If B Clicked Go to Basket
            }
            if (!gamepad1.b) {
                gamepad2BClicking = false;
            }

            if (gamepad1.x && !gamepad2XClicking) {
                gamepad2XClicking = true;
                armSystem.cycleSubmersible(); // If X Clicked Go to Submersible
            }
            if (!gamepad1.x) {
                gamepad2XClicking = false;
            }

            if (gamepad1.y && !gamepad2YClicking) {
                gamepad2YClicking = true;
                armSystem.cycleChamberTop(); // If Y Clicked Go to Top Chamber
            }
            if (!gamepad1.y) {
                gamepad2YClicking = false;
            }
        }

        // Set power to motors
        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);

        // Arm System Controls

        armSystem.periodicUpdate(telemetry);

        // Wrist and Gripper Values
        if (gamepad1.left_trigger > 0.5) {
            wristReference = 1;
        }
        if (gamepad1.right_trigger > 0.5) {
            wristReference = 0.3;
        }
        if (gamepad2.dpad_up) {
            gripperReference = 0.75;
        }
        if (gamepad2.dpad_down) {
            gripperReference = 0.15;
        }
        if (gamepad2.left_bumper) {
            armSystem.enterZeroPos();
        }

        LSLower.setPosition(wristReference - lolclock);
        LSTop.setPosition(gripperReference - lolclock);

        // Telemetry for debugging
        SparkFunOTOS.Pose2D pos = odometry.getPosition();
        telemetry.addData("OdoX", pos.x);
        telemetry.addData("OdoY", pos.y);
        telemetry.addData("Angle", pos.h);
        telemetry.addData("Gripper Position", LSTop.getPosition());
        telemetry.addData("Wrist Position", LSLower.getPosition());
        telemetry.addData("Extendo Reference", armSystem.getExtendoReference());
        telemetry.addData("Capstan Reference", armSystem.getCapstanReference());
        telemetry.update();
    }
}
