package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robot_utilities.FlyWheel;
import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot_utilities.Vals;

@TeleOp(name = "EchoOp")
public class EchoOp extends OpMode {
    private GamePadController gamepad;
    private Motor driveLeft, driveRight;
    private FlyWheel flywheel;
    private Servo hitter;
    private Motor intake1, intake2;

    private double intakeSpeed = 0;
    private double flywheelSpeed = 0;

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
//        flywheel.setRunMode(Motor.RunMode.VelocityControl);
//        flywheel.setVeloCoefficients(Vals.flywheel_kp, Vals.flywheel_ki, Vals.flywheel_kd);
//        flywheel.setFeedforwardCoefficients(0, 0.03);

        hitter = hardwareMap.servo.get("sv");
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
//            Vals.flywheel_direction *= -1;
        }

        if(gamepad.isBRelease()) {
            if(flywheel.isOn()) {
                flywheel.off();
            } else {
                flywheel.on();
            }
//            if(flywheelSpeed != 0) {
//                flywheel.setRunMode(Motor.RunMode.RawPower);
////                flywheelRunMode = "RAW";
//                flywheelSpeed = 0;
//            } else {
//                flywheel.setRunMode(Motor.RunMode.VelocityControl);
////                flywheelRunMode = "VelocityControl";
//                flywheelSpeed = Vals.flywheel_speed;
//            }
        }

        if(gamepad1.left_bumper) {
            hitter.setPosition(Vals.hitter_end);
        }
        else {
            hitter.setPosition(Vals.hitter_start);
        }

        driveLeft.set(leftSpeed);
        driveRight.set(rightSpeed);

        intake1.set(intakeSpeed);
        intake2.set(intakeSpeed);

//        flywheel.set(Vals.flywheel_direction * flywheelSpeed);
        flywheel.set();

        telemetry.addData("Flywheel Speed", flywheel.flywheel.get());
        telemetry.addData("Flywheel Set Speed", flywheelSpeed);
        telemetry.addData("Flywheel Velocity", flywheel.flywheel.encoder.getRawVelocity());
        telemetry.addData("Flywheel Position", flywheel.flywheel.getCurrentPosition());
        telemetry.addData("Hitter Position", hitter.getPosition());
        telemetry.addData("Left Speed", leftSpeed);
        telemetry.addData("Right Speed", rightSpeed);
        telemetry.addData("Intake Speed", intakeSpeed);


    }
}