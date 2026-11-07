package org.firstinspires.ftc.teamcode;

import com.pedropathing.utils.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teamPedroPathing.Constants;

@TeleOp
public class BiobuzzTeleOpMk1 extends OpMode {
    private RobotHardware robotHardware;
    private boolean shooterState = false;
    private Follower follower;
    private Timer timer;
    private final double TICKS_PER_REV = 28.0;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        robotHardware = new RobotHardware(this);
        timer = new Timer();
    }

    @Override
    public void loop() {
        follower.manual(
                -gamepad1.left_stick_y,
                gamepad1.right_stick_x,
                -gamepad1.left_stick_x
        );
        follower.update();

        if (gamepad1.yWasPressed()) {
            shooterState = !shooterState;
            if (shooterState) {
                timer.reset();
            }
        }

        if (shooterState) {
            robotHardware.shooterON();
        } else {
            robotHardware.shooterOFF();
        }

        if (gamepad1.dpadUpWasPressed()) {
            robotHardware.intakeSpeedUp();
        }
        if (gamepad1.dpadDownWasPressed()) {
            robotHardware.intakeSpeedDown();
        }

        // Priority order: Outtake (B) / Intake (A) / Auto-Transfer (Y/Shooter) / Off
        if (gamepad1.b) {
            robotHardware.outtakeOn();
        } else if (gamepad1.a) {
            robotHardware.intakeOn();
            robotHardware.transferOn2();
        } else if (shooterState && timer.seconds() >= 3.5) {
            robotHardware.transferOn();
            timer.reset();
        }else if (timer.seconds() >= 1.5) {
            robotHardware.intakeOff();
            }
            double currentTicksPerSec = robotHardware.shooterMotor.getVelocity();
            double currentRpm = (currentTicksPerSec / TICKS_PER_REV) * 60.0;
            telemetry.addData("Intake Velocity", robotHardware.power);
            telemetry.addData("Controls", "A = Intake | B = Reverse | Y = Toggle Shooter");
            telemetry.addData("Target Velocity (ticks/s)", currentTicksPerSec);
            telemetry.addData("Calculated Speed (RPM)", "%.1f RPM", currentRpm);
            telemetry.update();
        }
    }
