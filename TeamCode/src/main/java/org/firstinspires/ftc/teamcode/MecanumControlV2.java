package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Mecanum_Control_V2", group="Testier")

public class MecanumControlV2 extends OpMode {

    MecanumDrive robot    = new MecanumDrive();
    Shooter shooter       = new Shooter();
    Intake  intake        = new Intake();
    WobbleGrabber grabber = new WobbleGrabber();

    double driveSpeed;
    double turnSpeed;
    double direction;
    double shooterPower = -1;

    boolean isShooterOn  = false;
    boolean isShooterOff = true;
    boolean wasPowerIncreased;
    boolean wasPowerDecreased;

    private ElapsedTime period  = new ElapsedTime();
    private double runtime = 0;

    @Override
    public void init() {
        robot.init(hardwareMap);
        shooter.init(hardwareMap);
        intake.init(hardwareMap);
        grabber.init(hardwareMap);

        msStuckDetectInit = 18000;
        msStuckDetectLoop = 18000;

        telemetry.addData("Hello","be ready");
        telemetry.addData("Loop_Timeout",msStuckDetectLoop);
        telemetry.update();
    }
    @Override
    public void loop() {
        telemetry.addData("isShooterOn", isShooterOn);
        telemetry.addData("isShooterOff", isShooterOff);
        telemetry.addData("Shooter Power", shooterPower);
        //telemetry.addData("Drive Speed",driveSpeed);
        //telemetry.addData("Direction",direction);
        //telemetry.addData("Turn Speed", turnSpeed);
        //telemetry.addData("LB",robot.getLBencoder());
        //telemetry.addData("RB",robot.getRBencoder());
        //telemetry.addData("LF",robot.getLFencoder());
        //telemetry.addData("RF",robot.getRFencoder());
        telemetry.update();

        //Speed control (turbo/slow mode) and direction of stick calculation
        if (gamepad1.left_bumper) {
            driveSpeed = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            turnSpeed = gamepad1.right_stick_x;
        }else if (gamepad1.right_bumper) {
            driveSpeed = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y)*.4;
            direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            turnSpeed = gamepad1.right_stick_x*.4;
        }else {
            driveSpeed = Math.hypot(-gamepad1.left_stick_x, -gamepad1.left_stick_y)*.7;
            direction = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            turnSpeed = gamepad1.right_stick_x*.7;
        }
        //set power and direction of drive motors
        if (turnSpeed == 0) {
            robot.MecanumController(driveSpeed,direction,0);
        }

        //control of turning
        if (gamepad1.right_stick_x != 0 && driveSpeed == 0) {
            robot.leftFront.setPower(-turnSpeed);
            robot.leftBack.setPower(-turnSpeed);
            robot.rightFront.setPower(turnSpeed);
            robot.rightBack.setPower(turnSpeed);
        }

        //Turn shooter on
        if (gamepad2.a && !isShooterOn) {
            isShooterOn = true;
        }else if (!gamepad2.a && isShooterOn) {
            isShooterOff = false;
            shooter.shooterPower(shooterPower);
        }
        //Turn shooter off
        if (gamepad2.a && !isShooterOff) {
            isShooterOn = false;
        }else if (!gamepad2.a && !isShooterOn) {
            isShooterOff = true;
            shooter.shooterPower(0);
        }
        //Change shooter power
        if (gamepad2.dpad_up) {
            wasPowerIncreased = true;
        }else if (!gamepad2.dpad_up && wasPowerIncreased) {
            shooterPower -= .05;
            if (shooterPower <= -1.04) {
                shooterPower = -.65;
            }
            wasPowerIncreased = false;
        }
        if (gamepad2.dpad_down) {
            wasPowerDecreased = true;
        }else if (!gamepad2.dpad_down && wasPowerDecreased) {
            shooterPower += .05;
            if (shooterPower >= -.61) {
                shooterPower = -1;
            }
            wasPowerDecreased = false;
        }

        //Control intake latch servo
        if (gamepad2.dpad_right) {
            intake.intakeLatch.setPosition(.5);
        }else if (gamepad2.dpad_left) {
            intake.intakeLatch.setPosition(1);
        }

        //Control intake and transition
        if (gamepad2.left_bumper) {
            //independent control
            intake.intake.setPower(-gamepad2.right_stick_y);
            intake.transition.setPower(-gamepad2.left_stick_y);
        }else {
            //synchronous control
            intake.intake.setPower(-gamepad2.left_stick_y);
            intake.transition.setPower(-gamepad2.left_stick_y);
        }

        //Control grabber wrist
        if (gamepad1.dpad_left) {
            grabber.gripWrist.setPosition(.72);
        }else if (gamepad1.dpad_right) {
            grabber.gripWrist.setPosition(.23);
        }

        //Control grabber servo
        if (gamepad1.dpad_down) {
            grabber.gripperPosition(.77);
        }else if (gamepad1.dpad_up) {
            grabber.gripperPosition(0);
        }
    }
}
