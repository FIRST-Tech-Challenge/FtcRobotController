package org.firstinspires.ftc.teamcode;

import com.bravenatorsrobotics.core.FtcGamePad;
import com.bravenatorsrobotics.drive.MecanumDrive;
import com.bravenatorsrobotics.operation.TeleopMode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="Teleop")
public class Teleop extends TeleopMode<MecanumDrive> {

    // TODO: Replace with tested values
    private static final int LIFT_STAGE_1 = 100;
    private static final int LIFT_STAGE_2 = 200;
    private static final int LIFT_STAGE_3 = 300;

    private static final double LIFT_POWER = 0.75;

    private static final double CUP_OBJECT_THRESHOLD_CM = 6.0; // CM
    private static final double REDUCE_SPEED_MULTIPLIER = 0.25;

    private Config config;

    private DcMotorEx lift;
    private DcMotorEx intake;
    private DcMotorEx turnTableSpinner;

    // TODO: Check to see what servo is on bot and put it into the configuration
    private Servo cupServo;
    private TouchSensor liftTouchSensor;
    private RevColorSensorV3 cupDistanceSensor;

    private boolean shouldOverrideSpeedReduction = false;
    private boolean shouldFreeLiftWhenDone = false;

    private double turnTablePower = 1;

    // Create TeleopMode with specified specifications
    public Teleop() { super(new Specifications()); }

    @Override
    public void OnInitialize() {
        PrintControls();

        config = new Config(hardwareMap.appContext);

        // Reverse the turn table power if on red alliance
        if(config.allianceColor == Config.AllianceColor.RED)
            turnTablePower = -turnTablePower;

        lift = robot.GetMotor("lift", false);
        intake = robot.GetMotor("intake", false);
        turnTableSpinner = robot.GetMotor("turnTable", false);

        cupServo = hardwareMap.servo.get("cupServo");

        liftTouchSensor = hardwareMap.touchSensor.get("liftTouchSensor");

        cupDistanceSensor = hardwareMap.get(RevColorSensorV3.class, "cupDistanceSensor");
    }

    @Override
    public void OnStart() {

    }

    @Override
    public void OnUpdate() {
        HandleGamePadDrive();

        // Check to see if object is in cup. If so tilt it back.

        // Free Lift
        if(shouldFreeLiftWhenDone) {
            if(!lift.isBusy()) {
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
    }

    @Override
    public void OnStop() {
        robot.drive.Stop();
    }

    private void HandleGamePadDrive() {
        double v = Math.pow(gamepad1.left_stick_y, 3);
        double h = Math.pow(gamepad1.left_stick_x, 3) + Math.pow(driverGamePad.getLeftTrigger(), 3)
                - Math.pow(driverGamePad.getRightTrigger(), 3);
        double r = Math.pow(gamepad1.right_stick_x, 3);

        if(!liftTouchSensor.isPressed() && !shouldOverrideSpeedReduction) {
            v *= REDUCE_SPEED_MULTIPLIER;
            h *= REDUCE_SPEED_MULTIPLIER;
            r *= REDUCE_SPEED_MULTIPLIER;
        }

        super.robot.drive.Drive(v, h, r);
    }

    @Override
    protected void OnDriverGamePadChange(FtcGamePad gamePad, int button, boolean pressed) {
        switch (button) {
            case FtcGamePad.GAMEPAD_BACK:
                if(pressed) shouldOverrideSpeedReduction = !shouldOverrideSpeedReduction;
                break;
        }
    }

    @Override
    protected void OnOperatorGamePadChange(FtcGamePad gamePad, int button, boolean pressed) {
        switch (button) {

            // Turntable Direction Override
            case FtcGamePad.GAMEPAD_BACK:
                if(pressed) {
                    turnTablePower = -turnTablePower;
                }

                break;

            // Turntable Spinner
            case FtcGamePad.GAMEPAD_Y:

                if(pressed) {
                    turnTableSpinner.setPower(turnTablePower);
                } else {
                    turnTableSpinner.setPower(0);
                }

                break;

            // Intake
            case FtcGamePad.GAMEPAD_A:
                if(pressed) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }

                break;

            // Toggle Cup
            case FtcGamePad.GAMEPAD_B:
                // TODO: Toggle the cup dumped or not
                break;

            // Automatic Lift Down
            case FtcGamePad.GAMEPAD_DPAD_DOWN:
                if(pressed) {
                    lift.setTargetPosition(0);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(LIFT_POWER);
                    shouldFreeLiftWhenDone = true;

                    // TODO: Check for manual safety switch
                }

                break;

            // Automatic Lift Position 1
            case FtcGamePad.GAMEPAD_DPAD_LEFT:
                if(pressed) {
                    lift.setTargetPosition(LIFT_STAGE_1);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(LIFT_POWER);
                    shouldFreeLiftWhenDone = true;
                }

                break;

            // Automatic Lift Position 2
            case FtcGamePad.GAMEPAD_DPAD_UP:
                if(pressed) {
                    lift.setTargetPosition(LIFT_STAGE_2);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(LIFT_POWER);
                    shouldFreeLiftWhenDone = true;
                }

                break;

            // Automatic Lift Position 3
            case FtcGamePad.GAMEPAD_DPAD_RIGHT:
                if(pressed) {
                    lift.setTargetPosition(LIFT_STAGE_3);
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setPower(LIFT_POWER);
                    shouldFreeLiftWhenDone = true;
                }

                break;
        }
    }

    private boolean IsObjectInCup() {
        double distance = cupDistanceSensor.getDistance(DistanceUnit.CM);
        return (distance < CUP_OBJECT_THRESHOLD_CM) || (cupDistanceSensor.getLightDetected() > 0.1);
    }

    private void PrintControls() {
        telemetry.log().add("DRIVER CONTROLS");
        telemetry.log().add("(BACK) Emergency Movement Override");
        telemetry.addLine();

        telemetry.log().add("OPERATOR CONTROLS");
        telemetry.log().add("(BACK) Reverse Turn-Table Direction");
        telemetry.log().add("(Y) Turn-Table");
        telemetry.log().add("(A) Intake");
        telemetry.log().add("(B) Toggle Cup Position");
        telemetry.log().add("(D-Pad Down) Lift Down");
        telemetry.log().add("(D-Pad Left) Lift First Position");
        telemetry.log().add("(D-Pad Up) Lift Second Position");
        telemetry.log().add("(D-Pad Right) Lift Third Position");
    }

}
