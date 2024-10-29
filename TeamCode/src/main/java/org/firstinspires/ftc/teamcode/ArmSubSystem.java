package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.sun.tools.javac.util.Warner;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmSubSystem {
    private armPose positionalState = armPose.REST;
    private armPoseZone positionalZone = armPoseZone.OTHER;
    private DcMotor cap;
    private DcMotor extendo;
    private RevTouchSensor ClasslimitSwitch;
    private final PIDController capstanPID = new PIDController();
    private final PIDController extendoPID = new PIDController();
    private int capstanReference = 0;
    private int extendoReference = 0;
    private float wristReference = 0.1F;
    private float clawReference = 0.1F;
    private double lolclock;
    private Telemetry telemetry;
    // Servos
    private ServoImplEx wrist;
    private ServoImplEx claw;

    public ArmSubSystem(armPose positionalStateInitial, DcMotor capstanMotor, DcMotor spindleMotor, RevTouchSensor limitSwitch, ServoImplEx clawLinearServo, ServoImplEx wristLinearServo, Telemetry telemetry) {
        claw = clawLinearServo;
        wrist = wristLinearServo;
        positionalState = positionalStateInitial;
        cap = capstanMotor;
        extendo = spindleMotor;
        ClasslimitSwitch = limitSwitch;
        this.telemetry = telemetry;
        cap.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        cap.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendo.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendo.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extendoPID.init(0.008F, 0, 0.001F);
        capstanPID.init(0.007F, 0, 0.001F);
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
            case SUBMERSIBLE_B:
                positionalZone = armPoseZone.SUBMERSABLE;
                extendoReference = 2250;
                capstanReference = -117;
                break;
            case SUBMERSIBLE_A:
                positionalZone = armPoseZone.SUBMERSABLE;
                extendoReference = 2250;
                capstanReference = -170;
                break;
            case CHAMBER_B:
                positionalZone = armPoseZone.CHAMBER;
                extendoReference = 0;
                capstanReference = -400;
                break;
            case CHAMBER_A:
                positionalZone = armPoseZone.CHAMBER;
                extendoReference = 0;
                capstanReference = -600;
                break;
            case BASKET:
                positionalZone = armPoseZone.OTHER;
                extendoReference = 2250;
                capstanReference = -600;
                break;
            case REST:
                positionalZone = armPoseZone.OTHER;
                extendoReference = 0;
                capstanReference = -200;
                break;
            case ZERO:
                positionalZone = armPoseZone.OTHER;
                extendoReference = 0;
                capstanReference = 0;
        }
    }
    public void setManipulatorReference(wristState stateOfWrist, clawState stateOfClaw) {
        if (stateOfWrist == wristState.UP) {
            wristReference = 0.75f;
        } else {
            wristReference = 0f;
        }
        if (stateOfClaw == clawState.CLOSED) {
            clawReference = 0.1f;
        } else {
            clawReference = 1f;
        }
    }
    public void periodicUpdate(TelemetryPacket packet) {
        packet.put("ExtendoPosition", extendo.getCurrentPosition());
        packet.put("ExtendoReference", extendoReference);
        packet.put("CapstanPosition", cap.getCurrentPosition());
        packet.put("CapstanReference", capstanReference);
        float extendoPIDValue = extendoPID.getOutput(extendo.getCurrentPosition(), extendoReference);
        if (extendoPIDValue > 0.75) {
            extendo.setPower(0.75);
        } else if (extendoPIDValue < -0.75) {
            extendo.setPower(-0.75);
        } else {
            extendo.setPower(extendoPIDValue);
        }
        if (!ClasslimitSwitch.isPressed()) {
            float capPIDValue = capstanPID.getOutput(cap.getCurrentPosition(), capstanReference);
            if (capPIDValue > 0.75) {
                cap.setPower(0.75);
            } else if (capPIDValue < -0.75) {
                cap.setPower(-0.75);
            } else {
                cap.setPower(capPIDValue);
            }
        }
        if (lolclock > 0.05) {
            lolclock = lolclock - 0.001;
        } else {
            lolclock = lolclock + 0.001;
        }
        if (wristReference > 0.5) {
            wrist.setPosition(wristReference - lolclock);
        } else {
            wrist.setPosition(wristReference + lolclock);
        }
        if (clawReference > 0.5) {
            claw.setPosition(clawReference - lolclock);
        } else {
            claw.setPosition(clawReference + lolclock);
        }
        if (cap != null) {
            telemetry.addData("CapstanError", (capstanReference - cap.getCurrentPosition()));
        }
        if (extendo != null) {
            telemetry.addData("ExtendoError", (extendoReference - extendo.getCurrentPosition()));
        }
    }
    public void cycleSubmersible() {
        if (positionalZone != armPoseZone.SUBMERSABLE) {
            setReferences(armPose.SUBMERSIBLE_A);
        } else if (positionalState == armPose.SUBMERSIBLE_A) {
            setReferences(armPose.SUBMERSIBLE_B);
        } else {
            setReferences(armPose.REST);
        }
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

    public void enterZeroPos() {
        setReferences(armPose.ZERO);
    }
    public boolean isAtReference(TelemetryPacket packet) {
        if (cap != null) {
            int capInt = cap.getCurrentPosition();
            int extendoInt = extendo.getCurrentPosition();
            boolean b = (extendoInt < (30 + extendoReference)) && (extendoInt > (extendoReference - 30));
            boolean a = (capInt < (30 + capstanReference)) && (capInt > (capstanReference - 30));
            telemetry.addData("CapAcceptable", a);
            telemetry.addData("ExtendoAcceptable", b);
            packet.put("ExtendoAcceptable", b);
            packet.put("CapAcceptable", a);
            return a && b;
        }
        return false;
    }
}