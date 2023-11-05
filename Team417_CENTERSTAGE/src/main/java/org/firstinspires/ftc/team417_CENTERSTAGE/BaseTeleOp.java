package org.firstinspires.ftc.team417_CENTERSTAGE;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp League 1")
public class BaseTeleOp extends BaseOpMode {
    @Override
    public void runOpMode() {
        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            driveUsingControllers(false);
            outputUsingControllers();
            intakeUsingControllers();
            telemetry.addData("FRMotor", FR.getCurrentPosition());
            telemetry.addData("FRMotor", FR.getPowerFloat());
            telemetry.addData("FLMotor", FL.getCurrentPosition());
            telemetry.addData("FLMotor", FL.getPowerFloat());
            telemetry.addData("BRMotor", BR.getCurrentPosition());
            telemetry.addData("BRMotor", BR.getPowerFloat());
            telemetry.addData("BLMotor", BL.getCurrentPosition());
            telemetry.addData("BLMotor", BL.getPowerFloat());
            telemetry.addData("ArmMotor", armMotor.getCurrentPosition());
            telemetry.addData("DumperServo", dumperServo.getPosition());
            telemetry.update();
        }
    }

    public boolean sensitive = false;
    public boolean rightBumperIsPressed = false;

    public void driveUsingControllers() {
        if (!rightBumperIsPressed && gamepad1.right_bumper) {
            sensitive = !sensitive;
        }
        rightBumperIsPressed = gamepad1.right_bumper;

        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;

        if (sensitive) {
            sensitivity = 0.5;
            rotSensitivity = 0.8;
        } else {
            sensitivity = 1;
            rotSensitivity = 1;
        }

        double x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
        double y = curveStick(-gamepad1.left_stick_y) * sensitivity;
        double rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;

        mecanumDrive(x, y, rot);
    }

    public void driveUsingControllers(boolean curve) {
        if (!rightBumperIsPressed && gamepad1.right_bumper) {
            sensitive = !sensitive;
        }
        rightBumperIsPressed = gamepad1.right_bumper;

        double sensitivity, rotSensitivity;
        double strafeConstant = 1.1;

        if (sensitive) {
            sensitivity = 0.5;
            rotSensitivity = 0.8;
        } else {
            sensitivity = 1;
            rotSensitivity = 1;
        }

        double x, y, rot;
        if (curve) {
            x = curveStick(gamepad1.left_stick_x) * strafeConstant * sensitivity;
            y = curveStick(-gamepad1.left_stick_y) * sensitivity;
            rot = curveStick(gamepad1.right_stick_x) * rotSensitivity;
        } else {
            x = gamepad1.left_stick_x * strafeConstant * sensitivity;
            y = -gamepad1.left_stick_y * sensitivity;
            rot = gamepad1.right_stick_x * rotSensitivity;
        }
        mecanumDrive(x, y, rot);
    }

    public void intakeUsingControllers() {
        double speed = gamepad1.left_trigger - gamepad1.right_trigger;
        telemetry.addData("Left Trigger", gamepad1.left_trigger);
        telemetry.addData("Right Trigger", gamepad1.right_trigger);
        telemetry.addData("Intake Speed", speed);
        runIntakeMechanism(speed);
    }

    public void outputUsingControllers() {
        //controlArmUsingControllers();
        controlDumperUsingControllers();
        controlGateUsingControllers();
        armControlBackup();
    }

    private double armLocation = 0;  //where the arm should currently move to.
    private boolean dpadDownPressed = false;
    private boolean dpadUpPressed = false;
    private void armControlBackup(){
        double rStickSensitivity = 20; //how much moving right stick will effect the movement of the arm.
        double armVelocity = -gamepad2.right_stick_y * rStickSensitivity; //how fast the arm moves based off of the right stick and sensitivity.
        double[] backupArmPositions = new double[] {ARM_MOTOR_MIN_POSITION, ARM_MOTOR_MAX_POSITION / 2.0,
                                                    ARM_MOTOR_MAX_POSITION * (3.0/4.0), ARM_MOTOR_MAX_POSITION}; //array of arm positions the dpad can move to.
        armLocation += armVelocity; //change the current arm position by the velocity.

        /*if (armLocation < ARM_MOTOR_MIN_POSITION) {
            armLocation = ARM_MOTOR_MIN_POSITION;
        }
        if (armLocation > ARM_MOTOR_MAX_POSITION) {
            armLocation = ARM_MOTOR_MAX_POSITION;
        }*/

        if (gamepad2.dpad_up && gamepad2.dpad_down)
            ; //if both dpad buttons are being pressed do nothing.
        else if (gamepad2.dpad_up && !dpadUpPressed){ //if dpad up is being pressed change the arm location to the next location in the arm positions array.
            for (int i = 0; i < backupArmPositions.length; i++)
            {
                if (backupArmPositions[i] > armLocation) {
                    armLocation = backupArmPositions[i];
                    break;
                }
            }
        } else if (gamepad2.dpad_down && !dpadDownPressed){ //if dpad down is being pressed change the arm location to the last location in the arm positions array.
            for (int i = backupArmPositions.length - 1; i >= 0; i--)
            {
                if (backupArmPositions[i] > armLocation) {
                    armLocation = backupArmPositions[i];
                    break;
                }
            }
        }

        dpadDownPressed = gamepad2.dpad_down;
        dpadUpPressed = gamepad2.dpad_up;


        if (armMotor.getCurrentPosition() < armLocation + 50 && armMotor.getCurrentPosition() > armLocation - 50) {
            armMotor.setPower(0);
        } else if (armMotor.getCurrentPosition() > armLocation) {
            armMotor.setPower(-0.5);
        } else if (armMotor.getCurrentPosition() < armLocation) {
            armMotor.setPower(0.5);
        }

        telemetry.addData("arm target position", armLocation);
        //armMotor.setTargetPosition((int) Math.round(armLocation)); //tell the motor to move to arm location.
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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

    public boolean dumperTilted = false;
    public boolean xIsPressed = false;
    public boolean dumperDumped = false;
    public boolean bIsPressed = false;

    public void controlDumperUsingControllers() {
        /*
        double dumperSpeed = -gamepad2.right_stick_y;

        if (isEpsilonEquals(dumperSpeed, 0)) {
            moveDumper(DumperAction.STOPPING);
        } else if (dumperSpeed > 0) {
            moveDumper(DumperAction.DUMPING);
        } else if (dumperSpeed < 0) {
            moveDumper(DumperAction.RESETTING);
        }
        */

        if (!bIsPressed && gamepad2.b) {
            if (dumperDumped) {
                resetDumper();
                dumperTilted = false;
            } else {
                dumpDumper();
                dumperTilted = true;
            }
            dumperDumped = !dumperDumped;
        }
        bIsPressed = gamepad2.b;

        if (!xIsPressed && gamepad2.x) {
            if (dumperTilted) {
                resetDumper();
                dumperDumped = false;
            } else {
                dumpDumper();
                dumperDumped = true;
            }
            dumperTilted = !dumperTilted;
        }
        xIsPressed = gamepad2.x;
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
