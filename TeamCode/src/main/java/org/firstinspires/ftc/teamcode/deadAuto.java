package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Disabled
@Autonomous(name="deadAutoI44", group="Auto", preselectTeleOp="crank")
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

    static class goToFromFor {
        private ElapsedTime timer;
        private int from;
        private int to;
        private int forV;
        public void start(int fromVal, int toVal, int forVal) {
            timer = new ElapsedTime();
            timer.milliseconds();
            from = fromVal;
            to = toVal;
            forV = forVal;
        }
        public int getValue() {
            return (int) (from + ((to - from) * (timer.milliseconds()/forV)));
        }
    }

    enum armPose {
        BASKET,
        SUBMERSIBLE_A,
        SUBMERSIBLE_B,
        REST,
        CHAMBER_A,
        CHAMBER_B,
        ZERO,
        CUSTOM_1

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
        private static int capstanReference = 300;
        private static int extendoReference = 0;
        public void init(armPose positionalStateInitial, DcMotor capstanMotor, DcMotor spindleMotor, RevTouchSensor limitSwitch) {
            positionalState = positionalStateInitial;
            cap = capstanMotor;
            extendo = spindleMotor;
            ClasslimitSwitch = limitSwitch;
            extendoPID.init(0.005F, 0, 0.001F);
            capstanPID.init(0.0035F, 0, 0.0005F);
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
                case CUSTOM_1:
                    positionalZone = armPoseZone.OTHER;
                    extendoReference = 0;
                    capstanReference = 300;
                    break;
                case ZERO:
                    positionalZone = armPoseZone.OTHER;
                    extendoReference = 0;
                    capstanReference = 0;
                    break;
                case SUBMERSIBLE_B:
                    positionalZone = armPoseZone.SUBMERSABLE;
                    extendoReference = 1000;
                    capstanReference = 87;
                    break;
                case SUBMERSIBLE_A:
                    positionalZone = armPoseZone.SUBMERSABLE;
                    extendoReference = 2000;
                    capstanReference = 150;
                    break;
                case CHAMBER_B:
                    positionalZone = armPoseZone.CHAMBER;
                    extendoReference = 1000;
                    capstanReference = 100;
                    break;
                case CHAMBER_A:
                    positionalZone = armPoseZone.CHAMBER;
                    extendoReference = 1000;
                    capstanReference = 400;
                    break;
                case BASKET:
                    positionalZone = armPoseZone.OTHER;
                    extendoReference = 2300;
                     capstanReference = 750;
                    break;
                case REST:
                    positionalZone = armPoseZone.OTHER;
                    extendoReference = 0;
                    capstanReference = 150;
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
            telemetry.addData("Capstan Power", cap.getPower());
            telemetry.addData("Capstan Pos", cap.getCurrentPosition());
            telemetry.addData("Extendo Pos", extendo.getCurrentPosition());
            telemetry.addData("Capstan Encoder Status", cap.getController().getConnectionInfo());
            telemetry.addData("Exteno Encoder Status", extendo.getController().getConnectionInfo());
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
        ArmControlSubsystem.init(armPose.CUSTOM_1, cap, spindle, lswitch);
        LSTop.setPosition(0.3);
        LSLower.setPosition(0.1);
    }

    @Override
    public void start() {
        timeAtStart = System.currentTimeMillis();
        ArmSubSystem.cap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmSubSystem.cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmSubSystem.extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ArmSubSystem.extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void loop() {
        if (lolclock > 0.05) {
            lolclock = lolclock - 0.001;
        } else {
            lolclock = lolclock + 0.001;
        }
        double x, y, rx;
        double delay = 9000;
        if ((System.currentTimeMillis() - timeAtStart) < (delay + 3640)) {
            ArmControlSubsystem.setReferences(armPose.CUSTOM_1);
            LSTop.setPosition(0.3 - lolclock);
            LSLower.setPosition(0.9 - lolclock);
            x = 0;
            y = 0;
            rx = 0;
        } else if ((System.currentTimeMillis() - timeAtStart) < (delay + 5000)) {
            ArmControlSubsystem.setReferences(armPose.CHAMBER_A);
            LSTop.setPosition(0.3 - lolclock);
            LSLower.setPosition(0.9 - lolclock);
            x = 0;
            y = -0.25;
            rx = 0;
        } else if ((System.currentTimeMillis() - timeAtStart) < (delay + 5500)) {
            ArmControlSubsystem.setReferences(armPose.CHAMBER_B);
            LSTop.setPosition(0.3 - lolclock);
            LSLower.setPosition(0.9 - lolclock);
            x = 0;
            y = 0;
            rx = 0;
        } else if ((System.currentTimeMillis() - timeAtStart) < (delay + 8000)) {
            ArmControlSubsystem.setReferences(armPose.CHAMBER_B);
            LSTop.setPosition(0.3 - lolclock);
            LSLower.setPosition(0.9 - lolclock);
            x = 0;
            y = 0.1;
            rx = 0;
        } else if ((System.currentTimeMillis() - timeAtStart) < (delay + 11000)) {
            ArmControlSubsystem.setReferences(armPose.CHAMBER_B);
            LSTop.setPosition(0.9 - lolclock);
            LSLower.setPosition(0.9 - lolclock);
            x = 0;
            y = 0.05;
            rx = 0;
        } else if ((System.currentTimeMillis() - timeAtStart) < (delay + 17000)) {
            ArmControlSubsystem.setReferences(armPose.CHAMBER_B);
            LSTop.setPosition(0.9 - lolclock);
            LSLower.setPosition(0.9 - lolclock);
            x = 0;
            y = 0.2;
            rx = 0;
        }  else if ((System.currentTimeMillis() - timeAtStart) < (delay + 22000)) {
            ArmControlSubsystem.setReferences(armPose.REST);
            LSTop.setPosition(0.9 - lolclock);
            LSLower.setPosition(0.9 - lolclock);
            x = -0.5;
            y = 0.2;
            rx = 0;
        } else {
            LSTop.setPosition(0.9 - lolclock);
            LSLower.setPosition(0.1 - lolclock);
            ArmControlSubsystem.setReferences(armPose.ZERO);
            x = 0;
            y = 0.2;
            rx = 0;
        }

        ArmControlSubsystem.periodicUpdate(telemetry);

        // Set power to motors
        frontLeftMotor.setPower(y + x + rx);
        backLeftMotor.setPower(y - x + rx);
        frontRightMotor.setPower(y - x - rx);
        backRightMotor.setPower(y + x - rx);
    }
}
