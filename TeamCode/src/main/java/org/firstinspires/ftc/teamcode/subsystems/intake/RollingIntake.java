package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;

public class RollingIntake extends SonicSubsystemBase {

    private CRServo leftServo;

    private CRServo rightServo;

    private Servo elbowServo;


    NormalizedColorSensor colorSensor;

    private Telemetry telemetry;

    GamepadEx gamepad;

    private DriverFeedback feedback;

    private IntakeState state;

    private enum IntakeState { Hold, Intake, Outtake }

    public RollingIntake(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback) {
        /* instantiate motors */
        this.rightServo  = hardwareMap.get(CRServo.class,"RightIntake");
        this.leftServo  = hardwareMap.get(CRServo.class,"LeftIntake");

        this.elbowServo = hardwareMap.get(Servo.class, "Elbow");

        this.colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");

        this.gamepad = gamepad;
        this.telemetry = telemetry;
        this.feedback = feedback;

        this.elbowServo.setPosition(1);
        this.SetElbowInInStart();

        state = IntakeState.Hold;
    }

    @Override
    public void periodic() {
        super.periodic();

        double d = GetDepth();

        //telemetry.addData("distance", d);
        //telemetry.addData("IsDelivery", this.isInDeliveryPosition);

        if(d < 50) {
            if (feedback != null) {
                feedback.TurnLedGreen();
            }
        } else {
            if (feedback != null) {
                feedback.TurnLedRed();
            }
        }

        if(state == IntakeState.Intake) {
            //telemetry.addData("State", "Intake");

            if(d > 50 || this.isInDeliveryPosition) {
                telemetry.addData("power", 1);

                this.leftServo.setPower(-1);
                this.rightServo.setPower(1);

            } else {
                this.leftServo.setPower(-0.2);
                this.rightServo.setPower(0.2);
                //telemetry.addData("power",0);

                if (feedback != null) {
                    feedback.DriverRumbleBlip();
                    feedback.OperatorRumbleBlip();
                }
            }
        } else if (state == IntakeState.Outtake) {
            //telemetry.addData("State", "Outtake");

            if(d > 50) {
                if (feedback != null) {
                    feedback.DriverRumbleBlip();
                    feedback.OperatorRumbleBlip();
                }
            }
            this.leftServo.setPower(1);
            this.rightServo.setPower(-1);

        } else {
            //telemetry.addData("State", "Hold");

            this.leftServo.setPower(0);
            this.rightServo.setPower(0);
        }

        //telemetry.update();
    }

    boolean isInDeliveryPosition = false;

    public void SetInDeliveryPositionn(boolean isInDeliveryPosition) {
        this.isInDeliveryPosition = isInDeliveryPosition;
    }

    public void SetElbowInSpecimenPosition() {
        this.elbowServo.setPosition(0);
    }

    public void SetElbowInIntakePosition() {
        this.elbowServo.setPosition(.45);
    }

    public void SetElbowInInStart() {
        this.elbowServo.setPosition(1);
    }

    boolean isElbowInIntake = true;

    public void ToggleElbowPosition() {
        if(isElbowInIntake) {
            SetElbowInSpecimenPosition();
        } else {
            SetElbowInIntakePosition();
        }

        isElbowInIntake = !isElbowInIntake;
    }

    public void Intake() {
        state = IntakeState.Intake;
    }

    public void Outtake() {
        state = IntakeState.Outtake;
    }

    public void Hold(){
        state = IntakeState.Hold;
    }

    public double GetDepth() {
        if (colorSensor instanceof DistanceSensor) {
            double depth = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM);
            //telemetry.addData("Left distance (mm)", "%.3f", depth);
            return depth;
        }

        return 1000000;
    }

    public void IntakeInAuto() {
        this.leftServo.setPower(-1);
        this.rightServo.setPower(1);
    }

    public void OuttakeInAuto() {
        telemetry.addLine("Trying to outtake");
        telemetry.update();
        this.leftServo.setPower(1);
        this.rightServo.setPower(-1);
    }

    public void HoldInAuto() {
        telemetry.addLine("Holding out/intake");
        telemetry.update();

        this.leftServo.setPower(0);
        this.rightServo.setPower(0);
    }
}