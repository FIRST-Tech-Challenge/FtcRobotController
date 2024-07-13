package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Bot extends Component {
    private Intake intake = null;
    protected Lift lift = null;
    protected boolean liftAuto = true;
    private Servo shoulderL = null;
    private Servo shoulderR = null;
    private Servo wrist = null;
    private Dropper dropper = null;
    private Servo launcher = null;
    public static boolean handlerDeployed = true;
    public static boolean handlerDeploying = false;
    public static boolean handlerRetracting = false;
    public static boolean dropperDeployed = true;
    private double shoulderRDeploy1Pos = 0.83;
    private double shoulderRLoadPos = 0.375;
    private double shouldRGroundPos = 0.75;
    private double shoulderLDeploy1Pos = 0.83;
    private double shoulderLLoadPos = 0.375;
    private double shoulderLGroundPos = 0.75;
    private double wristDeploy1Pos = 0.7;
    private double wristLoadPos = 0.125;
    private double wristGroundPos = 0.0;
    private double launcherLockPos = 0.1;
    private double launcherUnlockPos = 0.45 ;
    private boolean loading = false;
    ElapsedTime timer;

    public Bot(HardwareMap hardwareMap, Telemetry telemetry, boolean loggingOn) {
        super(telemetry, false);
        // Intake
        intake = new Intake(hardwareMap, telemetry, loggingOn);

        // Lift
        lift = new Lift(hardwareMap, telemetry, loggingOn);
        liftAuto = true;

        // Shoulder
        shoulderL = hardwareMap.get(Servo.class, "shoulderL");
        shoulderR = hardwareMap.get(Servo.class, "shoulderR");
        shoulderL.setDirection(Servo.Direction.REVERSE);
        shoulderR.setDirection(Servo.Direction.FORWARD);

        // Wrist
        wrist = hardwareMap.get(Servo.class, "wrist");
        wrist.setDirection(Servo.Direction.REVERSE);

        // Dropper
        dropper = new Dropper(hardwareMap, telemetry, loggingOn);

        handlerDeployed = true;
        handlerDeploying = false;
        handlerRetracting = false;
        dropperDeployed = true;

        // Launcher
        launcher = hardwareMap.get(Servo.class, "launcher");
        launcher.setDirection(Servo.Direction.FORWARD);
        launcherLock();

        // Timer
        timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public void dropPixel() {
        dropper.dropPixel();
    }

    public void handlerRetract() {
        lift.goToRetractPosition();
        handlerRetracting = true;
    }

    private void dropperRetract() {
        wrist.setPosition(wristLoadPos);
        shoulderL.setPosition(shoulderLLoadPos);
        shoulderR.setPosition(shoulderRLoadPos);
        timer.reset();
    }

    public void liftToCruisePosition() {
        lift.goToCruisePosition();
    }

    public void liftToMinPosition() {
        lift.goToMinPosition();
    }
    private void liftLoad() {
        lift.goToLoadPosition();
    }

    public void tuckDropper()
    {
        wrist.setPosition(wristLoadPos);
        shoulderL.setPosition(shoulderLDeploy1Pos);
        shoulderR.setPosition(shoulderLDeploy1Pos);
        lift.goToMinPosition();
    }
    public void handlerDeployLevel1() {
        dropperDeployed = false;
        handlerDeployed = false;
        lift.goToDeploy1Position();
        handlerDeploying = true;
    }

    private void dropperDeploy() {
        wrist.setPosition(wristDeploy1Pos);
        shoulderL.setPosition(shoulderLDeploy1Pos);
        shoulderR.setPosition(shoulderRDeploy1Pos);
    }

    public void dropperToGroundPlacementPosition() {
        wrist.setPosition(wristGroundPos);
        shoulderL.setPosition(shoulderLGroundPos);
        shoulderR.setPosition(shouldRGroundPos);
    }
    public void liftToGroundPlacementPosition()
    {
        lift.goToGroundPlacementPosition();
    }

    public void load() {
        if (!handlerDeployed) {
            wrist.setPosition(wristLoadPos);
            dropper.load();
            intakeDeploy();
            loading = true;
        }
    }

    public void stopLoad() {
        intakeRetract();
        dropper.stopLoad();
        loading = false;
    }

    public void liftManualUp(double power) {
        if (handlerDeployed) {
            liftAuto = false;
            lift.manualUp(power);
        }
    }

    public void liftManualDown(double power) {
        if (handlerDeployed) {
            liftAuto = false;
            lift.manualDown(power);
        }
    }

    public void liftStop() {
        if (!liftAuto) {
            lift.stop();
            liftAuto = true;
        }
    }

    public void launcherLock() {
        launcher.setPosition(launcherLockPos);
    }

    public void launcherUnlock() {
        launcher.setPosition(launcherUnlockPos);
    }

    public void intakeDeploy() {
        intake.deploy();
    }

    public void intakeRetract() {
        intake.retract();
    }

    public void intakeForward() {
        intake.forward();
    }

    public void intakeReverse() {
        intake.reverse();
    }

    public void intakeOff() {
        intake.stop();
    }

    public void update() {
        if (loggingOn) {
            telemetry.addData("handlerDeploying: ", handlerDeploying);
            telemetry.addData("handlerDeployed: ", handlerDeployed);
            telemetry.addData("handlerRetracting: ", handlerRetracting);
            telemetry.addData("dropperDeployed: ", dropperDeployed);
            telemetry.addData("Time: ", timer.milliseconds());
        }
        if (liftAuto) {
            lift.update();
        }
        if (!lift.isBusy()) {
            if (handlerDeploying) {
                dropperDeploy();
                dropperDeployed = true;
                handlerDeploying = false;
                handlerDeployed = true;
            } else if (handlerRetracting) {
                if (dropperDeployed) {
                    dropperRetract();
                    dropperDeployed = false;
                } else if (timer.milliseconds() > 350) {
                    liftLoad();
                    handlerDeployed = false;
                    handlerRetracting = false;
                }
            }
        }

        intake.update();
        dropper.update();
        if (dropper.fullyLoaded() && loading) {
            loading = false;
            stopLoad();
        }
        telemetry.update();
    }
}
