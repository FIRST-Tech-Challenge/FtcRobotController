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

        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "ColorSensor");

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

        //double d = GetDepth();


        //telemetry.addData("distance", d);
        //telemetry.update();

        if(state == IntakeState.Intake) {
            //if(d > 40) {
                this.leftServo.setPower(-1);
                this.rightServo.setPower(1);
            //} else {
            //    Hold();
                //feedback.DriverRumbleBlip();
                //feedback.OperatorRumbleLeft();
            //    telemetry.addLine("Auto stop");
            //}
        } else if (state == IntakeState.Outtake) {
            //if(d > 130) {
                //feedback.DriverRumbleBlip();
                //feedback.OperatorRumbleLeft();
            //}
            this.leftServo.setPower(1);
            this.rightServo.setPower(-1);

        } else {
            this.leftServo.setPower(0);
            this.rightServo.setPower(0);
        }
    }

    public void SetElbowInSpecimenPosition() {
        this.elbowServo.setPosition(.3);
    }

    public void SetElbowInIntakePosition() {
        this.elbowServo.setPosition(.4);
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

 /*   public double GetDepth() {
        if (colorSensor instanceof DistanceSensor) {
            double depth = ((DistanceSensor) colorSensor).getDistance(DistanceUnit.MM);
            //telemetry.addData("Left distance (mm)", "%.3f", depth);
            return depth;
        }

        return 1000000;
    }*/

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
        telemetry.addLine("Trying to outtake");
        telemetry.update();

        this.leftServo.setPower(0);
        this.rightServo.setPower(0);
    }
}