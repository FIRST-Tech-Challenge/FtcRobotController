package org.firstinspires.ftc.teamcode.xendy;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.utils.DriveChassis;
import org.firstinspires.ftc.teamcode.utils.Numbers;
import org.firstinspires.ftc.teamcode.utils.controller.Controller;
import org.firstinspires.ftc.teamcode.utils.controller.GameController;
import org.firstinspires.ftc.teamcode.utils.controller.PowerCurve;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.stream.Collectors;

@TeleOp(name="XendyNotSimpleAutonomous")
public class XendyNotSimpleAutonomous extends OpMode {
    private final String RECORDED_DATA = "";

    private DriveChassis chassis;
    private double maxSpeed = 50;
    private final static float HORIZONTAL_BALANCE = 1.1f;

    private YawPitchRollAngles orientation;
    private double yaw;
    private double yawRad;
    private double normalizedYaw;

    private double targetRotation;

    // Gamepad States
    private final GameController controller1 = new GameController();
    private final GameController controller2 = new GameController();

    private final double SPEED_CHANGE_PER_PRESS = 5;

    boolean clawClosed = false;
    boolean bucketDown = false;

    ArrayList<SaveState> states = new ArrayList<>();
    @Override
    public void init() {
        chassis = new DriveChassis(this);
        if (!RECORDED_DATA.isEmpty()) {
            states = Arrays.stream(RECORDED_DATA.split("\\|")).map(this::loadStateFromString).collect(Collectors.toCollection(ArrayList::new));
            chassis.scoringArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chassis.collectionArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            chassis.endPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            chassis.scoringArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.collectionArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            chassis.endPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // change these to be reasonable later
            chassis.scoringArmMotor.setVelocity(0);
            chassis.collectionArmMotor.setVelocity(0);
            chassis.endPivotMotor.setVelocity(0);

            resetRuntime();
        }
    }

    boolean DONE_RECORDING = false;
    boolean SENT_TO_TELEMETRY = false;
    @Override
    public void loop() {
        controller1.update(gamepad1);
        controller2.update(gamepad2);
        if (controller1.pressed(Controller.Button.Start)) {
            DONE_RECORDING = true;
        }
        if (DONE_RECORDING) {
            if (!SENT_TO_TELEMETRY) {
                createSerializedStates();
                telemetry.addData("Data", SERIALIZED_FINAL_STATES);
                telemetry.update();
                SENT_TO_TELEMETRY = true;
            }
            return;
        }
        orientation = chassis.imu.getRobotYawPitchRollAngles();
        yaw = orientation.getYaw();
        yawRad = orientation.getYaw(AngleUnit.RADIANS);
        normalizedYaw = Numbers.normalizeAngle(yaw);

        float moveXInput = controller1.axis(Controller.Axis.LeftStickX, PowerCurve.Quadratic);
        float moveYInput = controller1.axis(Controller.Axis.LeftStickY, PowerCurve.Quadratic);
        float rotationInput = controller1.axis(Controller.Axis.RightStickX, PowerCurve.Cubic);

        if (controller1.pressed(Controller.Button.RightBumper))
            maxSpeed += SPEED_CHANGE_PER_PRESS;

        if (controller1.pressed(Controller.Button.LeftBumper))
            maxSpeed -= SPEED_CHANGE_PER_PRESS;
        maxSpeed = Range.clip(maxSpeed, 5, 90);
        if (controller1.stoppedChanging(Controller.Axis.RightStickX))
            targetRotation = normalizedYaw;
        double turnPower;
        if (rotationInput != 0) turnPower = rotationInput;
        else turnPower = Numbers.turnCorrectionSpeed(normalizedYaw, targetRotation);


        double verticalMovePower = moveXInput * Math.sin(-yawRad) + moveYInput * Math.cos(-yawRad);
        double horizontalMovePower = moveXInput * Math.cos(-yawRad) - moveYInput * Math.sin(-yawRad);
        if (RECORDED_DATA.isEmpty()) {
            if (controller2.pressed(Controller.Button.A)) {
                chassis.claw.setPosition(clawClosed ? 1 : 0);
                clawClosed = !clawClosed;
            }
            if (controller2.pressed(Controller.Button.B)) {
                chassis.bucket.setPosition(bucketDown ? 1 : 0);
                clawClosed = !bucketDown;
            }
            double xInput = controller2.axis(Controller.Axis.LeftStickX);
            if (chassis.collectionArmMotor.getCurrentPosition() >= 0 && xInput > 0 || chassis.collectionArmMotor.getCurrentPosition() <= -2150 && xInput < 0) {
                chassis.collectionArmMotor.setVelocity(0);
                telemetry.addLine("LIMIT REACHED FOR COLLECTION ARM");
            }
            else {
                chassis.collectionArmMotor.setVelocity(xInput * 500);
            }

            saveRobotState(horizontalMovePower, verticalMovePower, rotationInput != 0 ? normalizedYaw : targetRotation);
        }
        else {
            SaveState currentState = states.stream().filter(s->s.t<=getRuntime()).reduce((first, second) -> second).get();
            horizontalMovePower = currentState.mX;
            verticalMovePower = currentState.mY;
            turnPower = Numbers.turnCorrectionSpeed(normalizedYaw, currentState.yaw);
            chassis.scoringArmMotor.setTargetPosition(currentState.verticalSlidePosition);
            chassis.collectionArmMotor.setTargetPosition(currentState.horizontalSlidePosition);
            chassis.endPivotMotor.setTargetPosition(currentState.pivotPosition);
            chassis.bucket.setPosition(currentState.bucketPosition);
            chassis.claw.setPosition(currentState.clawPosition);
        }
        horizontalMovePower *= HORIZONTAL_BALANCE;

        double denominator = Math.max(Math.abs(verticalMovePower) + Math.abs(horizontalMovePower) + Math.abs(turnPower), 1);

        double leftFPower = (verticalMovePower + horizontalMovePower + turnPower) / denominator;
        double leftBPower = (verticalMovePower - horizontalMovePower + turnPower) / denominator;
        double rightFPower = (verticalMovePower - horizontalMovePower - turnPower) / denominator;
        double rightBPower = (verticalMovePower + horizontalMovePower - turnPower) / denominator;

        double velocityScale = chassis.DRIVE_GEAR_RATIO * chassis.TICKS_PER_REVOLUTION * maxSpeed / chassis.WHEEL_CIRCUMFERENCE;

        chassis.leftFrontMotor.setVelocity(leftFPower * velocityScale);
        chassis.rightFrontMotor.setVelocity(rightFPower * velocityScale);
        chassis.leftBackMotor.setVelocity(leftBPower * velocityScale);
        chassis.rightBackMotor.setVelocity(rightBPower * velocityScale);
        telemetry.update();
    }

    public void saveRobotState(double horzPow, double vertPow, double cYaw) {
        states.add(new SaveState(horzPow, vertPow, cYaw));
    }

    private class SaveState {
        public double t;
        public double mX, mY, yaw;
        public int verticalSlidePosition;
        public int horizontalSlidePosition;
        public int pivotPosition;
        public double bucketPosition;
        public double clawPosition;
        SaveState(double horzPow, double vertPow, double cYaw) {
            t = getRuntime();
            mX = horzPow;
            mY = vertPow;
            yaw = cYaw;
            verticalSlidePosition = chassis.scoringArmMotor.getCurrentPosition();
            horizontalSlidePosition = chassis.collectionArmMotor.getCurrentPosition();
            pivotPosition = chassis.endPivotMotor.getCurrentPosition();
            bucketPosition = chassis.bucket.getPosition();
            clawPosition = chassis.claw.getPosition();

            telemetry.addLine("===== LAST SAVED SAVESTATE =====");
            telemetry.addData("time", t);
            telemetry.addData("mX", mX);
            telemetry.addData("mY", mY);
            telemetry.addData("yaw", yaw);
            telemetry.addData("horizSLide", horizontalSlidePosition);
            telemetry.addData("vertSlide", verticalSlidePosition);
            telemetry.addData("wrist", pivotPosition);
            telemetry.addData("claw", clawPosition);
            telemetry.addData("bucket", bucketPosition);
        }
    }

    public SaveState loadStateFromString(String s) {
        String[] splits = s.split(":");
        SaveState state = new SaveState(0, 0, 0);
        state.t = Double.parseDouble(splits[0]);
        state.mX = Double.parseDouble(splits[1]);
        state.mY = Double.parseDouble(splits[2]);
        state.yaw = Double.parseDouble(splits[3]);
        state.horizontalSlidePosition = Integer.parseInt(splits[4]);
        state.verticalSlidePosition = Integer.parseInt(splits[5]);
        state.pivotPosition = Integer.parseInt(splits[6]);
        state.clawPosition = Double.parseDouble(splits[7]);
        state.bucketPosition = Double.parseDouble(splits[8]);
        return state;
    }

    String SERIALIZED_FINAL_STATES = "";
    private void createSerializedStates() {
        SERIALIZED_FINAL_STATES =
                states.stream().map(s->String.format(
                        "%s:%s:%s:%s:%s:%s:%s:%s:%s",
                        s.t, s.mX, s.mY, s.yaw,
                        s.horizontalSlidePosition, s.verticalSlidePosition, s.pivotPosition,
                        s.clawPosition, s.bucketPosition
                        )).collect(Collectors.joining("|"));
    }
}
