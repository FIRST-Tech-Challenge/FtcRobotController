package org.firstinspires.ftc.teamcode.bots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class NoodleSpinnerBot extends GyroBot{
    public CRServo intake = null;
    public Servo inOut = null;

    final double retracted = 0;
    final double extended = 1;

    public final double[] inOutPositions = new double[]{0, 1, 0.8};
    public int inOutPosIndex = 0;

    boolean isIntakeSpinning = false;
    boolean shouldToggle = false;
    boolean shouldUpdateIntake = true;

    private long lastToggleDone9 = 0;
    private long timeSinceToggle9 = 0;
    private long lastToggleDone1 = 0;
    private long timeSinceToggle1 = 0;
    private long lastToggleDone4 = 0;
    private long timeSinceToggle4 = 0;

    public NoodleSpinnerBot(LinearOpMode opMode) {
        super(opMode);
    }

    @Override
    public void init(HardwareMap ahwMap) {
        super.init(ahwMap);

        intake = hwMap.get(CRServo.class, "intake");
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        inOut = hwMap.get(Servo.class, "inOut");
        inOutPosIndex = 0;
        inOut.setPosition(inOutPositions[inOutPosIndex]);
    }

    public void intakeToggle(boolean button) {
        timeSinceToggle9 = System.currentTimeMillis() - lastToggleDone9;
        if (button && timeSinceToggle9 > 300) {
            if (isIntakeSpinning) {
                isIntakeSpinning = false;
                lastToggleDone9 = System.currentTimeMillis();
            } else {
                isIntakeSpinning = true;
                lastToggleDone9 = System.currentTimeMillis();
            }
        }
    }
    protected void updateIntake() {
        if (shouldUpdateIntake) {
            if (isIntakeSpinning) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        }
    }

    public void controlExtension(boolean toggle) {
        timeSinceToggle1 = System.currentTimeMillis() - lastToggleDone1;
        if (toggle && timeSinceToggle1 > 200) {
            if (inOutPosIndex == 0) {
                goToInOutPosition(1);
            } else if (inOutPosIndex == 1) {
                goToInOutPosition(0);
            }
            lastToggleDone1 = System.currentTimeMillis();
        }
    }

    protected void toggleInOut() {
        timeSinceToggle4 = System.currentTimeMillis() - lastToggleDone4;
        if (shouldToggle && inOutPosIndex == 0 && timeSinceToggle4 > 300) {
            controlExtension(true);
            lastToggleDone4 = System.currentTimeMillis();
        } else if (shouldToggle && inOutPosIndex == 1 && timeSinceToggle4 > 800) {
            controlExtension(true);
            lastToggleDone4 = System.currentTimeMillis();
        }
    }
    public void simpleIntake(boolean toggle) {
        timeSinceToggle1 = System.currentTimeMillis() - lastToggleDone1;
        if (toggle && timeSinceToggle1 > 200) {
            if (inOutPosIndex == 0) {
                //spinning and out
                inOutPosIndex = 1;
                isIntakeSpinning = true;
                inOut.setPosition(inOutPositions[inOutPosIndex]);
            } else if (inOutPosIndex == 1) {
                //not spinning and in
                inOutPosIndex = 0;
                inOut.setPosition(inOutPositions[inOutPosIndex]);
                sleep(300);
                isIntakeSpinning = false;
            }
            lastToggleDone1 = System.currentTimeMillis();
        }
    }
    public void goToInOutPosition(int index) {
        inOutPosIndex = index;
        inOut.setPosition(inOutPositions[inOutPosIndex]);
    }

    protected void onTick() {
        updateIntake();
        toggleInOut();
        super.onTick();
    }
}
