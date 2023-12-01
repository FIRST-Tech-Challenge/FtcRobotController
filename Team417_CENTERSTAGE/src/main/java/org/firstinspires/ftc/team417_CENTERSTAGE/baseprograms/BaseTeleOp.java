package org.firstinspires.ftc.team417_CENTERSTAGE.baseprograms;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.team417_CENTERSTAGE.roadrunner.MecanumDrive;

public abstract class BaseTeleOp extends BaseOpMode {
    public MecanumDrive drive;

    @Override
    public void runOpMode() {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        initializeHardware();

        waitForStart();

        while (opModeIsActive()) {
            resetIMUIfNeeded();
            driveUsingControllers(false);

            drive.updatePoseEstimate();

            telemetry.addData("x", drive.pose.position.x);
            telemetry.addData("y", drive.pose.position.y);
            telemetry.addData("heading", drive.pose.heading);

            if (armMotor != null && dumperServo != null && gateServo != null) {
                outputUsingControllers();
                intakeUsingControllers();
                telemetry.addData("ArmMotor", armMotor.getCurrentPosition());
                telemetry.addData("DumperServo", dumperServo.getPosition());
                telemetry.addData("GateServo", gateServo.getPosition());
            }
            telemetry.addData("FRMotor", FR.getCurrentPosition());
            telemetry.addData("FRMotor", FR.getPowerFloat());
            telemetry.addData("FLMotor", FL.getCurrentPosition());
            telemetry.addData("FLMotor", FL.getPowerFloat());
            telemetry.addData("BRMotor", BR.getCurrentPosition());
            telemetry.addData("BRMotor", BR.getPowerFloat());
            telemetry.addData("BLMotor", BL.getCurrentPosition());
            telemetry.addData("BLMotor", BL.getPowerFloat());
            telemetry.update();
        }
    }

    boolean leftBumperIsPressed = false;

    public void resetIMUIfNeeded() {
        if (gamepad1.left_bumper && !leftBumperIsPressed) {
            drive.imu.resetYaw();
        }
        leftBumperIsPressed = gamepad1.left_bumper;
    }

    public boolean sensitive = false;

    public void driveUsingControllers() {
        sensitive = gamepad1.right_bumper;

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
        sensitive = gamepad1.right_bumper;

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
        controlArmUsingControllers();
        controlDumperUsingControllers();
        controlGateUsingControllers();
    }

    private double armTargetLocation = 0;  //where the arm should currently move to.
    private boolean dpadDownPressed = false;
    private boolean dpadUpPressed = false;
    private void controlArmUsingControllers(){
        double rStickSensitivity = 35; //how much moving right stick will effect the movement of the arm.
        double armVelocity = -gamepad2.right_stick_y * rStickSensitivity; //how fast the arm moves based off of the right stick and sensitivity.
        double[] armPositions = new double[] {ARM_MOTOR_MIN_POSITION, ARM_MOTOR_MAX_POSITION / 2.0,
                                                    ARM_MOTOR_MAX_POSITION * (3.0/4.0), ARM_MOTOR_MAX_POSITION}; //array of arm positions the dpad can move to.
        armTargetLocation += armVelocity; //change the current arm position by the velocity.

        /*if (armLocation < ARM_MOTOR_MIN_POSITION) {
            armLocation = ARM_MOTOR_MIN_POSITION;
        }
        if (armLocation > ARM_MOTOR_MAX_POSITION) {
            armLocation = ARM_MOTOR_MAX_POSITION;
        }*/

        if (gamepad2.dpad_up && gamepad2.dpad_down)
            ; //if both dpad buttons are being pressed do nothing.
        else if (gamepad2.dpad_up && !dpadUpPressed){ //if dpad up is being pressed change the arm location to the next location in the arm positions array.
            for (int i = 0; i < armPositions.length; i++)
            {
                if (armPositions[i] > armTargetLocation) {
                    armTargetLocation = armPositions[i];
                    break;
                }
            }
        } else if (gamepad2.dpad_down && !dpadDownPressed){ //if dpad down is being pressed change the arm location to the last location in the arm positions array.
            for (int i = armPositions.length - 1; i >= 0; i--)
            {
                if (armPositions[i] > armTargetLocation) {
                    armTargetLocation = armPositions[i];
                    break;
                }
            }
        }

        dpadDownPressed = gamepad2.dpad_down;
        dpadUpPressed = gamepad2.dpad_up;


        if (armMotor.getCurrentPosition() < armTargetLocation + 50 && armMotor.getCurrentPosition() > armTargetLocation - 50) {
            armMotor.setPower(0);
        } else if (armMotor.getCurrentPosition() > armTargetLocation) {
            armMotor.setPower(-0.7);
        } else if (armMotor.getCurrentPosition() < armTargetLocation) {
            armMotor.setPower(0.7);
        }

        telemetry.addData("arm target position", armTargetLocation);
        //armMotor.setTargetPosition((int) Math.round(armLocation)); //tell the motor to move to arm location.
        //armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

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

        if (armMotor.getCurrentPosition() >= ARM_MOTOR_DUMPER_HITTING_INTAKE_LOWER_BOUND && armMotor.getCurrentPosition() <= ARM_MOTOR_DUMPER_HITTING_INTAKE_UPPER_BOUND) {
            if (dumperServo.getPosition() > DUMPER_SERVO_TILT_POSITION) {
                tiltDumper();
            }
        } else if (armMotor.getCurrentPosition() < ARM_MOTOR_DUMPER_HITTING_INTAKE_LOWER_BOUND) {
            if (dumperServo.getPosition() < DUMPER_SERVO_RESET_POSITION) {
                resetDumper();
            }
        }

        if (!bIsPressed && gamepad2.b) {
            if (dumperDumped) {
                resetDumper();
                dumperDumped = false;
            } else {
                dumpDumper();
                dumperDumped = true;
            }
        }
        bIsPressed = gamepad2.b;

        /*
        if (!xIsPressed && gamepad2.x) {
            if (dumperTilted) {
                resetDumper();
                dumperDumped = false;
            } else {
                tiltDumper();
                dumperDumped = true;
            }
            dumperTilted = !dumperTilted;
        }
        xIsPressed = gamepad2.x;
        */
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
