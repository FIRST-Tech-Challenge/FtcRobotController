/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import android.widget.Switch;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import static java.lang.Thread.sleep;

public class AtlasRobot {

    private Telemetry telemetry;
    private LinearOpMode opMode;


    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor rightManipulator = null;
    private DcMotor leftManipulator = null;
    private Servo foundationLeft;
    private Servo foundationRight;
    public ManipulatorDirection manipulatorState;
    public boolean manipulatorAutostop = false;
    Servo capstoneArm;
    Servo capstoneDrop;
    Servo inRamp;
    DigitalChannel digitalTouch;
    enum ManipulatorDirection { IN, OUT, STOP}

    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        foundationLeft = hardwareMap.get(Servo.class, "foundationLeft");
        foundationRight = hardwareMap.get(Servo.class, "foundationRight");
        capstoneArm = hardwareMap.get(Servo.class, "capstone_arm");
        capstoneDrop = hardwareMap.get(Servo.class, "capstone_drop");
        inRamp = hardwareMap.get(Servo.class, "in_ramp");

        leftManipulator = hardwareMap.get(DcMotor.class, "left_manipulator");
        rightManipulator = hardwareMap.get(DcMotor.class, "right_manipulator");

        digitalTouch = hardwareMap.get(DigitalChannel.class, "switch");

        leftManipulator.setPower(0);
        rightManipulator.setPower(0);

        dropCapstone(false);
        foundationMover(true);
        inRamp.setPosition(0);

        digitalTouch.setMode(DigitalChannel.Mode.INPUT);

        // Reverse the motors that runs backwards when connected directly to the battery

    }

    void capstoneArm(boolean out) {
        if (out) {
            capstoneArm.setPosition(1);
        } else {
            capstoneArm.setPosition(0);
        }
    }
    void dropCapstone (boolean open) {
        if (open) {
            capstoneDrop.setPosition(.5);

        } else {
            capstoneDrop.setPosition(1);
        }
    }

    void foundationMover(boolean up ){
        if (up){
            foundationLeft.setPosition(.35);
            foundationRight.setPosition(.65);
        } else {
            foundationLeft.setPosition(.6);
            foundationRight.setPosition(.4);
        }
    }

    void setManipulator(ManipulatorDirection direction) {
        setManipulator(direction, false);
    }

    void setManipulator(ManipulatorDirection direction, boolean autoStop) {
        manipulatorAutostop = autoStop;
        manipulatorState=direction;
        switch (direction) {
            case IN:
                inRamp.setPosition(1);
                leftManipulator.setPower(-1);
                rightManipulator.setPower(1);
                break;
            case OUT:
                inRamp.setPosition(0);
                leftManipulator.setPower(1);
                rightManipulator.setPower(-1);
                break;
            case STOP:
                leftManipulator.setPower(0);
                rightManipulator.setPower(0);
                inRamp.setPosition(0);
                break;
        }
    }

    void manipulatorAutostop() {
        if (manipulatorAutostop && (manipulatorState != ManipulatorDirection.STOP)) {
            setManipulator(ManipulatorDirection.STOP);
        }
    }

    boolean switchPressed() {
        return !digitalTouch.getState();
    }

}
