package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robot.FlyWheel;
import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot.Hitter;

@TeleOp(name = "EchoOp")
public class EchoOp extends OpMode {
    private GamePadController gamepad;
    private Motor driveLeft, driveRight;
    private FlyWheel flywheel;
    private Hitter hitter;
    private Motor intake1, intake2;

    private double intakeSpeed = 0;
    private boolean flywheelOn = false;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        gamepad = new GamePadController(gamepad1);

        driveLeft = new Motor(hardwareMap, "dl");
        driveRight = new Motor(hardwareMap, "dr");
        driveLeft.setRunMode(Motor.RunMode.VelocityControl);
        driveRight.setRunMode(Motor.RunMode.VelocityControl);
        driveLeft.setVeloCoefficients(0.05, 0, 0);
        driveRight.setVeloCoefficients(0.05, 0, 0);

        intake1 = new Motor(hardwareMap, "in1");
        intake2 = new Motor(hardwareMap, "in2");
        intake1.setRunMode(Motor.RunMode.VelocityControl);
        intake2.setRunMode(Motor.RunMode.VelocityControl);
        intake1.setVeloCoefficients(0.05, 0, 0);
        intake2.setVeloCoefficients(0.05, 0, 0);

        flywheel = new FlyWheel(new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE));

        hitter = new Hitter(hardwareMap.servo.get("sv"));
    }

    @Override
    public void loop() {
        gamepad.update();


        double leftSpeed = -gamepad1.left_stick_y;
        double rightSpeed = gamepad1.right_stick_y;
        if(gamepad1.right_trigger >= 0.1) {
            intakeSpeed = 0.7;
        } else if(gamepad1.left_trigger >= 0.1) {
            intakeSpeed = -0.7;
        } else {
            intakeSpeed = 0;
        }

        if(gamepad.isARelease()) {
            flywheel.flipDirection();
        }

        if(gamepad.isBRelease()) {
            if(flywheel.isOn()) {
                flywheelOn = false;
            } else {
                flywheelOn = true;
            }
        }

        if(flywheelOn) {
            flywheel.on();
        } else {
            flywheel.off();
        }

        String isReady = "FLYWHEEL NOT READY";

        if(flywheel.isReady()) {
            isReady = "FLYWHEEL READY!";
        }


        if(gamepad1.left_bumper) {
            hitter.hit();
        }
        else {
            hitter.reset();
        }

        driveLeft.set(leftSpeed);
        driveRight.set(rightSpeed);

        intake1.set(intakeSpeed);
        intake2.set(intakeSpeed);


        telemetry.addData("Flywheel Speed", flywheel.flywheel.get());
        telemetry.addData("Flywheel Velocity", flywheel.flywheel.getCorrectedVelocity());
        telemetry.addData("Flywheel Position", flywheel.flywheel.getCurrentPosition());
        telemetry.addData("Hitter Position", hitter.hitter.getPosition());
        telemetry.addData("Left Speed", leftSpeed);
        telemetry.addData("Right Speed", rightSpeed);
        telemetry.addData("Intake Speed", intakeSpeed);
        telemetry.addData("Flywheel Ready State", isReady);


    }
}