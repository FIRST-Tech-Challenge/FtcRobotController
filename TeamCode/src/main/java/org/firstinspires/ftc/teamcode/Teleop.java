package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.bravenatorsrobotics.core.FtcGamePad;
import com.bravenatorsrobotics.drive.MecanumDrive;
import com.bravenatorsrobotics.operation.TeleopMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Teleop")
public class Teleop extends TeleopMode<MecanumDrive> {

    private static final double CUP_OBJECT_THRESHOLD_CM = 6.0; // CM

    private DcMotorEx lift;
    private DcMotorEx intake;
    private DcMotorEx turnTableSpinner;

    private Servo cupServo;

    private TouchSensor liftTouchSensor;

    private RevColorSensorV3 cupDistanceSensor;
    private ColorSensor cupColorSensor;

    // Create TeleopMode with specified specifications
    public Teleop() { super(new Specifications()); }

    @Override
    public void OnInitialize() {
        lift = robot.GetMotor("lift", false);
        turnTableSpinner = robot.GetMotor("turnTable", false);
        intake = robot.GetMotor("intake", false);

        cupServo = hardwareMap.servo.get("cupServo");

        liftTouchSensor = hardwareMap.touchSensor.get("liftTouchSensor");

        cupColorSensor = hardwareMap.colorSensor.get("cupDistanceSensor");
        cupDistanceSensor = hardwareMap.get(RevColorSensorV3.class, "cupDistanceSensor");
    }

    @Override
    public void OnStart() {

    }

    @Override
    public void OnUpdate() {
//        float[] hsvValues = { 0.0f, 0.0f, 0.0f };
//
//        Color.RGBToHSV(
//                (int) (cupColorSensor.red() * 255),
//                (int) (cupColorSensor.green() * 255),
//                (int) (cupColorSensor.blue() * 255),
//                hsvValues
//        );
//
//        telemetry.addData("HUE", hsvValues[0]);
//        telemetry.addData("S", hsvValues[1]);
//        telemetry.addData("V", hsvValues[2]);
//        telemetry.addData("Is In Cup", IsObjectInCup());
//        telemetry.update();

        // Cube the input to scale the power
        robot.drive.Drive(Math.pow(gamepad1.left_stick_y, 3), Math.pow(gamepad1.left_stick_x, 3), Math.pow(-gamepad1.right_stick_x, 3));



    }

    @Override
    public void OnStop() {
        robot.drive.Stop();
    }

    @Override
    protected void OnDriverGamePadChange(FtcGamePad gamePad, int button, boolean pressed) {

    }

    @Override
    protected void OnOperatorGamePadChange(FtcGamePad gamePad, int button, boolean pressed) {
        switch (button) {

            case FtcGamePad.GAMEPAD_DPAD_UP:
                if(pressed) { // Max encoder tick
                    lift.setPower(1);
                } else {
                    lift.setPower(0);
                }

                break;

            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed && !liftTouchSensor.isPressed()) { // if we aren't on the physical switch
                    lift.setPower(-1);
                } else {
                    lift.setPower(0);
                }

                break;

            case FtcGamePad.GAMEPAD_A:
                if(pressed) {
                    intake.setPower(-1);
                } else {
                    intake.setPower(0);
                }

                break;

            case FtcGamePad.GAMEPAD_Y:
                if(pressed) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }

                break;


        }
    }

    private boolean IsObjectInCup() {
        double distance = cupDistanceSensor.getDistance(DistanceUnit.CM);
        return (distance < CUP_OBJECT_THRESHOLD_CM) || (cupDistanceSensor.getLightDetected() > 0.1);
    }

}
