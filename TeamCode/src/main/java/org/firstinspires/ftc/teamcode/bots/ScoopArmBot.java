package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ScoopArmBot extends FourWheelDriveBot{
    public CRServo intake = null;
    public DcMotor scoopArm = null;

    boolean isSpinning = false;

    long lastToggleDone = 0;
    long timeSinceToggle = 0;

    long lastToggleDone2 = 0;
    long timeSinceToggle2 = 0;

    public ScoopArmBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);
        intake = hwMap.get(CRServo.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        scoopArm = hwMap.get(DcMotor.class, "scoopArm");
        scoopArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        scoopArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void intakeToggle(boolean button) {
        timeSinceToggle = System.currentTimeMillis() - lastToggleDone;
        if (button && timeSinceToggle > 200) {
            if (isSpinning) {
                intake.setPower(0);
                isSpinning = false;
            } else {
                intake.setPower(1);
                isSpinning = true;
            }
            lastToggleDone = System.currentTimeMillis();
        }
    }

    public void controlScoop(boolean up, boolean down) {
        timeSinceToggle2 = System.currentTimeMillis() - lastToggleDone2;
        if (up && timeSinceToggle2 > 200) {
            scoopArm.setTargetPosition(100);
            scoopArm.setPower(0.2);
            scoopArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lastToggleDone2 = System.currentTimeMillis();
        } else if (down && timeSinceToggle2 > 200) {
            scoopArm.setTargetPosition(0);
            scoopArm.setPower(0.2);
            scoopArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lastToggleDone2 = System.currentTimeMillis();
        }
    }

}
