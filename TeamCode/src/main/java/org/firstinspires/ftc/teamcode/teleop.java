package org.firstinspires.ftc.teamcode;

import android.graphics.Path;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.drivetrain;
import org.firstinspires.ftc.teamcode.GamepadStates;

@TeleOp(name = "Teleop", group = "Teleop")
public class teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double speed = .5;

        //Distance sensor set up
        DistanceSensor Distance;
        Distance = hardwareMap.get(DistanceSensor.class, "Distance");
        //finds and sorts distance from object
        double DistanceFrom = Distance.getDistance(DistanceUnit.CM);
        // Tell us distance from object
        telemetry.addData("Distance", DistanceFrom);
        telemetry.update();
         TouchSensor touch = null;


        Limelight3A limelight;


        drivetrain Drive = new drivetrain();
        Intake Intake = new Intake();
        Limelight Lime = new Limelight();

        GamepadStates newGamePad1 = new GamepadStates(gamepad1);
        GamepadStates newGamePad2 = new GamepadStates(gamepad2);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        touch = hardwareMap.touchSensor.get("touch");


        limelight.pipelineSwitch(0);

        limelight.start();

        Drive.init(this);
        Intake.init(this);
        Lime.init();

        waitForStart();

        while (opModeIsActive()) {
            newGamePad1.updateState();
            newGamePad2.updateState();

            if (gamepad1.left_stick_x > .4) {
                Drive.strafeRight(speed);
            } else if (gamepad1.left_stick_x < -.4) {
                Drive.strafeLeft(speed);
            } else if (gamepad1.left_stick_y < -.4) {
                Drive.forward(speed);
            } else if (gamepad1.left_stick_y > .4) {
                Drive.backward(speed);
            } else if (gamepad1.right_stick_x > .4) {
                Drive.turnRight(speed);
            } else if (gamepad1.right_stick_x < -.4) {
                Drive.turnLeft(speed);
            } else {
                if (!newGamePad2.a.state) {
                    Drive.stop();
                }
            }

            if (newGamePad1.right_bumper.released) {
                if (speed < 1) {
                    speed += .1;
                }
            } else if (newGamePad1.left_bumper.released) {
                if (speed > 0) {
                    speed -= .1;
                }
            }
            if (newGamePad1.b.pressed) {
                Drive.strafeLDistance(0.5, 24);
            }

            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    if (newGamePad2.a.state) {
                        if (result.getTx() > 10) {
                            while (result.getTx() > 10) {
                                Drive.forward(.25);
                            }
                            Drive.stop();
                        }
                        if (result.getTy() > 10) {
                            while (result.getTy() > 10) {
                                Drive.strafeRight(.25);
                            }
                            Drive.stop();
                        } else if (result.getTy() < -10) {
                            while (result.getTy() < -10) {
                                Drive.strafeLeft(.25);
                            }
                            Drive.stop();
                        }
                    } else {
                        Drive.stop();
                    }
                    Pose3D botpose = result.getBotpose();
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("Botpose", botpose.toString());
                    telemetry.update();

                }
            }
            //
            if (gamepad2.left_stick_y < -0.4&&!touch.isPressed()) {
                Intake.intake();
            } else if (gamepad2.left_stick_y > 0.4) {
                Intake.eject();
            } else {
                Intake.transport();
            }

            if (newGamePad1.a.released) {
                Drive.forwardDistance(.25, 24);
            }

        }
    }
}