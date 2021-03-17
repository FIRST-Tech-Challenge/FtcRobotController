package org.wheelerschool.robotics.comp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class CompBot {
    public WebcamName webcam;

    public DcMotor launchLeft;
    public DcMotor launchRight;
    public Servo launchPush;

    public Servo wobbleGrab;
    public DcMotor wobbleArm;

    public DcMotor driveFLeft;
    public DcMotor driveFRight;
    public DcMotor driveBLeft;
    public DcMotor driveBRight;

    public DcMotor intake;

    private ElapsedTime jiggleTime = new ElapsedTime();

    public enum IntakeMode {
        IN,
        OUT,
        STOP
    }

    public enum WobblePosition {
        GRAB,
        UP,
        STOWED
    }


    public CompBot(HardwareMap hw) {
        webcam = hw.get(WebcamName.class, "Webcam 1");

        launchLeft = hw.dcMotor.get("launchLeft");
        launchLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        launchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchRight = hw.dcMotor.get("launchRight");
        launchRight.setDirection(DcMotorSimple.Direction.FORWARD);
        launchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        launchPush = hw.servo.get("launchPush");

        wobbleGrab = hw.servo.get("wobbleGrab");

        wobbleArm = hw.dcMotor.get("wobbleArm");
        wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wobbleArm.setDirection(DcMotorSimple.Direction.FORWARD);

        driveFLeft  = hw.dcMotor.get("driveFLeft");
        driveFLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveFRight = hw.dcMotor.get("driveFRight");
        driveFRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveFRight.setDirection(DcMotorSimple.Direction.FORWARD);

        driveBLeft  = hw.dcMotor.get("driveBLeft");
        driveBLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        driveBRight = hw.dcMotor.get("driveBRight");
        driveBRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        driveBRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intake = hw.dcMotor.get("intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setDriveDirect(float fLeft, float fRight, float bLeft, float bRight) {
        driveFLeft.setPower(fLeft);
        driveFRight.setPower(fRight);
        driveBLeft.setPower(bLeft);
        driveBRight.setPower(bRight);
    }

    public void setDrive(float forward, float strafe, float rotate) {
        setDriveDirect(
                forward + rotate + strafe,
                forward - rotate - strafe,
                forward + rotate - strafe,
                forward - rotate + strafe
        );
    }

    public void setDriveMotorMode(DcMotor.RunMode mode) {
        driveFLeft.setMode(mode);
        driveFRight.setMode(mode);
        driveBLeft.setMode(mode);
        driveBRight.setMode(mode);
    }

    private void runDriveEncoder(float power, int fLeft, int fRight, int bLeft, int bRight) {
        setDriveDirect(0,0,0,0);
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        driveFLeft.setTargetPosition(fLeft);
        driveFRight.setTargetPosition(fRight);
        driveBLeft.setTargetPosition(bLeft);
        driveBRight.setTargetPosition(bRight);

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDriveDirect(power, power, power, power);

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setDriveEncTranslate(float power, float forward, float strafe) {

        int forwardEnc = (int) (forward * 10000f / 2830f);
        int strafeEnc = (int) (strafe * 10000f / 1940f);

        runDriveEncoder(power, forwardEnc + strafeEnc,
                forwardEnc - strafeEnc,
                forwardEnc - strafeEnc,
                forwardEnc + strafeEnc);
    }

    public void setDriveEncRotate(float power, int rotate) {
        runDriveEncoder(power, rotate, -rotate, rotate, -rotate);
    }

    public void jiggle() {
        // Yes
        float jiggleDir = 1;
        if (jiggleTime.milliseconds() > 100) {
            jiggleTime.reset();
        } else if (jiggleTime.milliseconds() > 50) {
            jiggleDir = -1;
        }

        setDrive(jiggleDir, 0, 0);
    }

    public boolean atDriveTarget() {
        return !driveBLeft.isBusy() && !driveBRight.isBusy() && !driveFLeft.isBusy() && !driveFRight.isBusy();
    }

    public void launcher(boolean mode) {
        float power = mode ? 1.0f : 0;

        launchLeft.setPower(power*0.9f);
        launchRight.setPower(power);
    }

    public void launchPush(boolean mode) {
        float pos = mode ? 0.65f : 0.1f;
        launchPush.setPosition(pos);
    }

    public void intakeMode(IntakeMode mode) {
        float power = 0f;

        if (mode == IntakeMode.IN) {
            power = 1.0f;
        } else if (mode == IntakeMode.OUT) {
            power = -0.5f;
        }

        intake.setPower(power);
    }

    public void setWobbleGrab(boolean grab) {
        if (grab) {
            wobbleGrab.setPosition(1f);
        } else {
            wobbleGrab.setPosition(0f);
        }
    }

    public void setWobbleArm(WobblePosition pos) {
        if (pos == WobblePosition.STOWED) {
            wobbleArm.setTargetPosition(-100);
        } else if (pos == WobblePosition.UP) {
            wobbleArm.setTargetPosition(-300);
        } else if (pos == WobblePosition.GRAB) {
            wobbleArm.setTargetPosition(-750);
        }
        wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        wobbleArm.setPower(0.5f);
    }

    public void stop() {
        launcher(false);
        setDriveDirect(0,0,0,0);
    }
}
