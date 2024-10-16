package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="secondAuto", group="Auto")
public class deadAuto extends OpMode {
    static class PIDController {
        private static double p;
        private static double i;
        private static double d;
        private static double integralSummation;
        private static double lastError;
        ElapsedTime timer;

        public static void setP(double pX) {
            p = pX;
        }

        public static void setI(double iX) {
            i = iX;
        }

        public static void setD(double dX) {
            d = dX;
        }

        public void init(float Xp, float Xi, float Xd) {
            p = Xp;
            i = Xi;
            d = Xd;
            timer = new ElapsedTime();
        }
        public double getOutput(double state, double reference) {
            double error = reference - state;
            integralSummation += (error * timer.seconds());
            double derivative = ((error - lastError) / timer.seconds());
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
                    capstanReference = 450;
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

    // Servos
    private ServoImplEx LSLower;
    private ServoImplEx LSTop;

    // Clock Funcs
    private double lolclock = 0.01;

    // ACS & DBS
    private ArmSubSystem ArmControlSubsystem =  new ArmSubSystem();
    private long timeAtStart;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void init() {
        // Motors & Sensors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        backRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        DcMotor spindle = hardwareMap.get(DcMotor.class, "spindle");
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

        // Breaking mode
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        cap.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Prep ArmControlSubsystem
        ArmControlSubsystem.init(MecanumTeleOp.armPose.REST, cap, spindle, lswitch);
        LSTop.setPosition(1);
    }

    @Override
    public void start() {
        timeAtStart = System.currentTimeMillis();
    }

    @Override
    public void loop() {
        if (lolclock > 0.05) {
            lolclock = lolclock - 0.001;
        } else {
            lolclock = lolclock + 0.001;
        }
        ArmControlSubsystem.periodicUpdate();
        double x, y, rx;
        if ((System.currentTimeMillis() - timeAtStart) < 3000) {
            x = 0;
            y = -0.3;
            rx = 0;
        } else if ((System.currentTimeMillis() - timeAtStart) < 4000) {
            ArmControlSubsystem.setReferences(MecanumTeleOp.armPose.CHAMBER_A);
            x = 0;
            y = 0;
            rx = 0;
        } else if ((System.currentTimeMillis() - timeAtStart) < 5000) {
            ArmControlSubsystem.setReferences(MecanumTeleOp.armPose.CHAMBER_B);
            x = 0;
            y = 0;
            rx = 0;
        } else if ((System.currentTimeMillis() - timeAtStart) < 5500) {
            ArmControlSubsystem.setReferences(MecanumTeleOp.armPose.CHAMBER_A);
            x = 0;
            y = 0.1;
            rx = 0;
        } else if ((System.currentTimeMillis() - timeAtStart) < 6000) {
            ArmControlSubsystem.setReferences(MecanumTeleOp.armPose.CHAMBER_A);
            LSTop.setPosition(1);
            x = 0;
            y = 0.3;
            rx = 0;
        }  else if ((System.currentTimeMillis() - timeAtStart) < 7000) {
            x = 0.5;
            y = 0;
            rx = 0;
        } else {
            x = 0.5;
            y = 0;
            rx = 0;
        }
        // Set power to motors
        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);
    }
}
