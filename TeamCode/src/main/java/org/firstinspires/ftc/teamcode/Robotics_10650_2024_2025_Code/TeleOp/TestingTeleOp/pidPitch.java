package org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.TeleOp.TestingTeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robotics_10650_2024_2025_Code.InitializeFolder.RobotInitialize;

@TeleOp(name = "pidPitch")
public class pidPitch extends LinearOpMode {

    // Run the initialize function
    RobotInitialize robot;

    int liftPitchPosition = 0;
    double maxLifEtxtension = 0;
    int liftExtenderPosition = 0;

    double p = 1;
    double i = 1;
    double d = -2.5;
    double f = 3;


    @Override
    public void runOpMode() throws InterruptedException {
// create and define the initialization variable
        robot = new RobotInitialize(this, false);


        // initialization of the control of the robot when start is pressed
        waitForStart();
        while (opModeIsActive()) {
            // controller inputs that is inputted by the drive team
            controllerInput();
        }
    }

    public void controllerInput() {
        if (Math.abs(robot.liftPitch.getCurrentPosition() - liftPitchPosition) > 50) {
            if (robot.liftPitch.getCurrentPosition() < liftPitchPosition) {
                robot.liftPitch.setVelocity(2150);
            } else if (robot.liftPitch.getCurrentPosition() >= liftPitchPosition) {
                robot.liftPitch.setVelocity(-2150);
                if (liftPitchPosition > 1500) {
                    robot.liftPitch.setVelocity(-3000);
                }
            }
        } else {
            robot.liftPitch.setVelocity(0);
        }

            if (liftPitchPosition <= 2325 && liftPitchPosition >= 0 ||
                    (liftPitchPosition >= 2325 && gamepad2.left_stick_y < 0) || // 3200 goes to the
                    // maximum horizontal position and further (try something less than this)
                    (liftPitchPosition <= 0 && gamepad2.left_stick_y > 0)) {
                if (liftExtenderPosition > maxLifEtxtension) {
                    liftExtenderPosition = (int) maxLifEtxtension;  //change to max lift xtension
                }
                //determines where the lift pitch goes
                if (gamepad2.left_stick_y < -0.2) {//going up

                    liftPitchPosition = liftPitchPosition - 35;
                    if (liftPitchPosition > 1500) {
                        liftPitchPosition = liftPitchPosition - 25;
                    }


                } else if (gamepad2.left_stick_y > 0.2) {//going down
                    liftPitchPosition = liftPitchPosition + 40;

                    if (liftPitchPosition < 0) {
                        liftPitchPosition = 0;
                    } else if (liftPitchPosition > 2300) {
                        liftPitchPosition = 2300;  //change to max lift xtension
                    }

                    double pitchAngle = robot.liftPitch.getCurrentPosition() * (90) / 2595;
                    if (pitchAngle >= 31.25) {
                        maxLifEtxtension = 1210 / (Math.sin(Math.toRadians(pitchAngle))); // horizontal bound
                    } else {
                        maxLifEtxtension = 5000;
                    }

            }
        }


                    if (gamepad2.dpad_up) {
                        p += 0.01;
                    }
                    if (gamepad2.dpad_down) {
                        i += 0.01;
                    }
                    if (gamepad2.dpad_right) {
                        d += 0.01;
                    }
                    if (gamepad2.dpad_left) {
                        f += 0.01;
                    }

                    if (gamepad2.triangle) {
                        p -= 0.01;
                    }
                    if (gamepad2.cross) {
                        i -= 0.01;
                    }
                    if (gamepad2.circle) {
                        d -= 0.01;
                    }
                    if (gamepad2.square) {
                        f -= 0.01;
                    }


                    robot.liftPitch.setVelocityPIDFCoefficients(p, i, d, f);

                    telemetry.addData("p", p);
                    telemetry.addData("d", d);
                    telemetry.addData("i", i);
                    telemetry.addData("f", f);
                    telemetry.addData("target pos", liftPitchPosition);
                    telemetry.addData("current pos", robot.liftPitch.getCurrentPosition());

                    telemetry.update();
                }
}