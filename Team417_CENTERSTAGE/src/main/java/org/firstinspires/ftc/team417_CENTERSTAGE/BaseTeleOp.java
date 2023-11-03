package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp League 1")
public class BaseTeleOp extends BaseOpMode {
    @Override
    public void runOpMode() {
        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            driveUsingControllers(true);
            //outputUsingControllers();
            //intakeUsingControllers();
        }
    }

    public void driveUsingControllers() {
        double sensitivity = 1;
        double rotSensitivity = 1;
        double strafeConstant = 1.1;

        double x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
        double y = curveStick(-gamepad1.left_stick_y) * sensitivity;
        double rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;

        mecanumDrive(x, y, rot);
    }

    public void driveUsingControllers(boolean curve) {
        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;
        double x, y, rot;
        if (curve) {
            sensitivity = 1;
            rotSensitivity = 1;
            x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
            y = curveStick(-gamepad1.left_stick_y) * sensitivity;
            rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;
        } else {
            sensitivity = 0.5;
            rotSensitivity = 0.8;
            x = gamepad1.left_stick_x * strafeConstant * sensitivity;
            y = -gamepad1.left_stick_y * sensitivity;
            rot = gamepad1.right_stick_x * rotSensitivity;
        }
        mecanumDrive(x, y, rot);
    }

    public void intakeUsingControllers() {
        double speed = gamepad1.left_trigger - gamepad1.right_trigger;
        runIntakeMechanism(speed);
    }

    public void outputUsingControllers() {
        controlArmUsingControllers();
        controlDumperUsingControllers();
        controlGateUsingControllers();
    }

    public double armPositionIndex;

    public void controlArmUsingControllers() {
        moveArm(-gamepad2.right_stick_y);

        for (int i = 0; i < armPositions.length; i++) {
            if (armMotor.getCurrentPosition() < armPositions[i]) {
                armPositionIndex = i - 0.5;
            } else if (isEpsilonEquals(armMotor.getCurrentPosition(), armPositions[i])) {
                armPositionIndex = i;
            }
        }

        if (isEpsilonEquals(armPositionIndex % 1.0, 0) || isEpsilonEquals(armPositionIndex % 1.0, 1)) {
            if (gamepad2.dpad_up && gamepad2.dpad_down) {
            } else if (gamepad2.dpad_up) {
                positionArm(((int) Math.round(armPositionIndex)) + 1);
            } else if (gamepad2.dpad_down) {
                positionArm(((int) Math.round(armPositionIndex)) - 1);
            }
        } else {
            if (gamepad2.dpad_up && gamepad2.dpad_down) {
            } else if (gamepad2.dpad_up) {
                positionArm((int) Math.ceil(armPositionIndex));
            } else if (gamepad2.dpad_down) {
                positionArm((int) Math.floor(armPositionIndex));
            }
        }
    }

    public boolean bIsPressed = false;

    public void controlDumperUsingControllers() {
        double dumperSpeed = -gamepad2.right_stick_y;

        if (isEpsilonEquals(dumperSpeed, 0)) {
            moveDumper(DumperAction.STOPPING);
        } else if (dumperSpeed > 0) {
            moveDumper(DumperAction.DUMPING);
        } else if (dumperSpeed < 0) {
            moveDumper(DumperAction.RESETTING);
        }

        if (!bIsPressed && gamepad2.b) {
            if (!isEpsilonEquals(dumperServo.getPosition(), 1.0)) {
                dumpDumper();
            } else {
                resetDumper();
            }
        }
        bIsPressed = gamepad2.b;
    }

    public boolean gateOpen = false;
    public boolean aIsPressed = false;

    public void controlGateUsingControllers() {
        if (!aIsPressed && gamepad2.a){
            if (gateOpen) {
                closeGate();
            } else {
                openGate();
            }
            gateOpen = !gateOpen;
        }
        aIsPressed = gamepad2.a;
    }

    //Adds stick curve to the joysticks
    public double curveStick(double rawSpeed) {
        double logSpeed;
        if (rawSpeed >= 0) {
            logSpeed = Math.pow(rawSpeed, 2);
        } else {
            logSpeed = -Math.pow(rawSpeed, 2);
        }
        return logSpeed;
    }
}
