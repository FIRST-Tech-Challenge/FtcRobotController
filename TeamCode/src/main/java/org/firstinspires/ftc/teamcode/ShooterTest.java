package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robot_utilities.GamePadController;
import org.firstinspires.ftc.robot_utilities.Vals;


@TeleOp(name = "ShooterTest")
public class ShooterTest extends OpMode {
    private GamePadController gamepad;
    private Motor flywheel;
    private Servo hitter;
    private double speedModifier = 0;
    private boolean pressedA = false;
    private boolean pressedB = false;

    private double flywheelSpeed = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        gamepad = new GamePadController(gamepad1);
        flywheel = new Motor(hardwareMap, "fw", Motor.GoBILDA.BARE);
        flywheel.setRunMode(Motor.RunMode.VelocityControl);
        flywheel.setVeloCoefficients(Vals.flywheel_kp, Vals.flywheel_ki, Vals.flywheel_kd);
        flywheel.setFeedforwardCoefficients(0, 0.03);

        hitter = hardwareMap.servo.get("sv");

    }

    @Override
    public void loop() {
        gamepad.update();

        if(gamepad.isARelease()) {
            Vals.flywheel_direction *= -1;
        }

        if(gamepad.isBRelease()) {
            if(flywheelSpeed != 0) {
                flywheel.setRunMode(Motor.RunMode.RawPower);
//                flywheelRunMode = "RAW";
                flywheelSpeed = 0;
            } else {
                flywheel.setRunMode(Motor.RunMode.VelocityControl);
//                flywheelRunMode = "VelocityControl";
                flywheelSpeed = Vals.flywheel_speed;
            }
        }

        if(gamepad1.left_bumper) {
            hitter.setPosition(Vals.hitter_end);
        }
        else {
            hitter.setPosition(Vals.hitter_start);
        }


        flywheel.set(Vals.flywheel_direction * flywheelSpeed);

        telemetry.addData("Flywheel Speed", flywheel.get());
        telemetry.addData("Flywheel Set Speed", flywheelSpeed);
        telemetry.addData("Flywheel Velocity", flywheel.encoder.getRawVelocity());
        telemetry.addData("Flywheel Position", flywheel.getCurrentPosition());
        telemetry.addData("Hitter Position", hitter.getPosition());


    }
}