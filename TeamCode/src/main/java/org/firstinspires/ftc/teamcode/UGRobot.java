/*7078 mecanumm drive code */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class UGRobot {

    private Telemetry telemetry;
    private LinearOpMode opMode;


    enum MoveDirection {FORWARD, BACKWARD, LEFT, RIGHT}

    private DcMotor pickupbottom = null;
    private DcMotor pickuptop = null;
    private DcMotor pickup = null;
    private DcMotor shooter = null;
    private Servo launchServo;
    public pickupDirection pickupState;
    public shooterDirection shooterState;

    enum pickupDirection {IN, OUT, STOP}
    enum shooterDirection {IN, OUT, STOP, IDLE}


    public void init(HardwareMap hardwareMap, Telemetry telemetryIn, LinearOpMode opModeIn) {

        telemetry = telemetryIn;
        opMode = opModeIn;

        launchServo = hardwareMap.get(Servo.class,"servo");

        pickupbottom = hardwareMap.get(DcMotor.class, "em1");
        pickuptop = hardwareMap.get(DcMotor.class, "em2");
        shooter = hardwareMap.get(DcMotor.class,"em0");

        pickupbottom.setPower(0);
        pickuptop.setPower(0);
        setShooter(shooterDirection.IDLE);

    }

    public void setPickup(UGRobot.pickupDirection direction) {
        pickupState = direction;
        switch (direction) {
            case IN:
                pickupbottom.setPower(-1);
                pickuptop.setPower(-1);
                break;
            case OUT:
                pickupbottom.setPower(1);
                pickuptop.setPower(1);
                break;
            case STOP:
                pickupbottom.setPower(0);
                pickuptop.setPower(0);
                break;
        }
    }
    public void setLaunchServo (boolean in) {
        if(in) {
            launchServo.setPosition(0);
        } else {
            launchServo.setPosition(1);
        }
    }
    public void setShooter(UGRobot.shooterDirection direction) {
        shooterState = direction;
        switch (direction) {
            case IN:
                shooter.setPower(-1);
                break;
            case OUT:
                shooter.setPower(.7);
                break;
            case STOP:
                shooter.setPower(0);
                break;
            case IDLE:
                shooter.setPower(0.5);
                break;
        }
    }
}