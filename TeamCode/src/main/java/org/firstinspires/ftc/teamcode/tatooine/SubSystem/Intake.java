package org.firstinspires.ftc.teamcode.tatooine.SubSystem;


import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;

public class Intake {

    //add the variables
    private final double INTAKE_SPEED = 1;
    private final double OUTTAKE_SPEED = -INTAKE_SPEED;
    boolean outtaking = false;
    private boolean buttonPressed = false;
    private boolean isRed = false;
    private boolean isSpecimen;
    private boolean IS_DEBUG = false;
    private double power = 0;
    private CRServo intakeRight = null;
    private CRServo intakeLeft = null;
    private CheckAlliance alliance;
    private ColorSensorOur colorSensorOur;
    private Telemetry telemetry;
    private ElapsedTime timer = new ElapsedTime();

    //intake constructor
    public Intake(OpMode opMode, boolean isRed, boolean IS_DEBUG) {
        this.IS_DEBUG = IS_DEBUG;
        telemetry = opMode.telemetry;
        //colorSensorOur = new ColorSensorOur(opMode, IS_DEBUG);
        intakeRight = opMode.hardwareMap.get(CRServo.class, "IR");
        intakeLeft = opMode.hardwareMap.get(CRServo.class, "IL");
        this.isRed = isRed;
        telemetry.addData("isRed", isRed);
        telemetry.update();
        init();
        if (IS_DEBUG) {
            telemetry.addData("IntakeConstructor", true);
        }
    }

    public void init() {
        //TODO change directions if needed
        //intake.setDirection(CRServo.Direction.FORWARD);
        intakeLeft.setDirection(CRServo.Direction.REVERSE);
        if (IS_DEBUG) {
            telemetry.addData("IntakeInit", true);
        }
    }


    public double getINTAKE_SPEED() {
        return INTAKE_SPEED;
    }

    public double getOUTTAKE_SPEED() {
        return OUTTAKE_SPEED;
    }

    public boolean isOuttaking() {
        return outtaking;
    }

    public void setOuttaking(boolean outtaking) {
        this.outtaking = outtaking;
    }

    public boolean isButtonPressed() {
        return buttonPressed;
    }

    public void setButtonPressed(boolean buttonPressed) {
        this.buttonPressed = buttonPressed;
    }

    public boolean isRed() {
        return isRed;
    }

    public void setRed(boolean red) {
        isRed = red;
    }

    public boolean isSpecimen() {
        return isSpecimen;
    }

    public void setSpecimen(boolean specimen) {
        isSpecimen = specimen;
    }

    public boolean isIS_DEBUG() {
        return IS_DEBUG;
    }

    public void setIS_DEBUG(boolean IS_DEBUG) {
        this.IS_DEBUG = IS_DEBUG;
    }

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public CRServo getIntakeRight() {
        return intakeRight;
    }

    public void setIntakeRight(CRServo intakeRight) {
        this.intakeRight = intakeRight;
    }

    public CheckAlliance getAlliance() {
        return alliance;
    }

    public void setAlliance(CheckAlliance alliance) {
        this.alliance = alliance;
    }

    public ColorSensorOur getColorSensorOur() {
        return colorSensorOur;
    }

    public void setColorSensorOur(ColorSensorOur colorSensorOur) {
        this.colorSensorOur = colorSensorOur;
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void setTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public ElapsedTime getTimer() {
        return timer;
    }

    public void setTimer(ElapsedTime timer) {
        this.timer = timer;
    }

    public CRServo getIntakeLeft() {
        return intakeLeft;
    }

    public void setIntakeLeft(CRServo intakeLeft) {
        this.intakeLeft = intakeLeft;
    }

    //intake action that is active as long as i press a button
    public Action intake() {
        this.buttonPressed = buttonPressed;
        return new SetPowerAction(INTAKE_SPEED);
    }

    //outtake action that is active as long as i press a button
    public Action outtake() {
        power = OUTTAKE_SPEED;
        return new SetPowerAction(OUTTAKE_SPEED);
    }

    //intake action that intakes and outtake by right color
    public Action intakeByColor(boolean isSpecimen) {
        this.isSpecimen = isSpecimen;
        return new IntakeByColor();
    }

    public Action setPowerAction(double power) {
        return new SetPowerAction(power);
    }


    public class SetPowerAction implements Action {
        double power = 0;
        public SetPowerAction(double power) {
            this.power = power;
            timer.reset();
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            intakeRight.setPower(power);
            intakeLeft.setPower(power);
            return false;
        }
    }

    //intake action that intakes and outtake by right color
    public class IntakeByColor implements Action {


        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            boolean colorCheek = colorSensorOur.isRightColor(isSpecimen);
            double dis = colorSensorOur.getDistance();
            boolean isIn = dis < 2;

            if (!isIn && !outtaking) {
                intakeRight.setPower(INTAKE_SPEED);
                intakeLeft.setPower(INTAKE_SPEED);
            } else if (colorCheek) {
                intakeRight.setPower(0);
                intakeLeft.setPower(0);
            } else {
                intakeRight.setPower(OUTTAKE_SPEED);
                if (!outtaking) {
                    timer.reset();
                    outtaking = true;

                } else {
                    outtaking = timer.milliseconds() < 750;
                }
            }
            if (IS_DEBUG) {
                telemetry.addData("Color", colorCheek);
                telemetry.addData("Distance", dis);
                telemetry.addData("isSpecimen", isSpecimen);
                telemetry.addData("Speed", power);
                telemetryPacket.put("Color", colorCheek);
                telemetryPacket.put("Distance", dis);
                telemetryPacket.put("isSpecimen", isSpecimen);
                telemetryPacket.put("Speed", power);
            }
            return (!isIn && !colorCheek);
        }
    }
}