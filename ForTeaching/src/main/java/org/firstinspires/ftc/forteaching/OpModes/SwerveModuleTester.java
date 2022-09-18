package org.firstinspires.ftc.forteaching.OpModes;

import android.util.Log;
import android.util.Pair;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.forteaching.SwerveModule;

import java.util.List;

/*
Motors: GoBilda 5202/3/4:
0: leftRearMotor
1: leftFrontMotor
2: rightRearMotor
3: rightFrontMotor
Servos: CR Servos
0: rightRearServo
1: rightFrontServo
4: leftRearServo
5: leftFrontServo
Analog Inputs:
0: rightFrontEncoder
1: rightRearEncoder
2: leftFrontEncoder
3: leftRearEncoder
*/

@Disabled
@TeleOp(name = "PIDTuner")
public class SwerveModuleTester extends LinearOpMode {
    SwerveModule leftFront;
    SwerveModule rightFront;
    SwerveModule leftRear;
    SwerveModule rightRear;
    private List<LynxModule> hubs = null;

    public void hwinit() {
        // Set up 'bulk updates' mode...
        if (hubs == null) {
            hubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : hubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            leftFront = new SwerveModule(hardwareMap, "leftFront");
            rightFront = new SwerveModule(hardwareMap, "rightFront");
            leftRear = new SwerveModule(hardwareMap, "leftRear");
            rightRear = new SwerveModule(hardwareMap, "rightRear");
        }
    }

    public void encoderTestOpMode() throws InterruptedException {
        // PIDFCoefficients
        telemetry.log().setDisplayOrder(Telemetry.Log.DisplayOrder.NEWEST_FIRST);
        telemetry.log().setCapacity(6);
        telemetry.addData("State", "Getting hardware");
        telemetry.update();
        hwinit();
        telemetry.addData("State", "Waiting for start");
        telemetry.update();
        while (!isStarted()) {
            idle();
        }
        ElapsedTime et = new ElapsedTime();
        et.reset();
        while (opModeIsActive()) {
            telemetry.addData("LF", "%f", leftFront.getAngle());
            telemetry.addData("RF", "%f", rightFront.getAngle());
            telemetry.addData("LR", "%f", leftRear.getAngle());
            telemetry.addData("RR", "%f", rightRear.getAngle());
            telemetry.addData("Loop", et.toString());
            et.reset();
            telemetry.update();
        }
    }

    private static double degrees(double val) {
        while (val > 360) {
            val -= 360;
        }
        while (val < 0) {
            val += 360;
        }
        return val;
    }


    public void moduleTestOpMode() throws InterruptedException {
        hwinit();
        while (!isStarted()) idle();
        ElapsedTime et = new ElapsedTime();
        et.reset();
        double nextAngle = 45;
        while (opModeIsActive()) {

            if (gamepad1.triangle) {
                rightFront.setRotatePower(.5);
            } else {
                rightFront.setRotatePower(0);
            }
            if (gamepad1.dpad_up) {
                rightFront.setDrivePower(.3);
            } else {
                rightFront.setDrivePower(0);
            }

            if (gamepad1.square) {
                leftFront.setRotatePower(.5);
            } else {
                leftFront.setRotatePower(0);
            }
            if (gamepad1.dpad_left) {
                leftFront.setDrivePower(.3);
            } else {
                leftFront.setDrivePower(0);
            }

            if (gamepad1.cross) {
                leftRear.setRotatePower(.5);
            } else {
                leftRear.setRotatePower(0);
            }
            if (gamepad1.dpad_down) {
                leftRear.setDrivePower(.3);
            } else {
                leftRear.setDrivePower(0);
            }

            if (gamepad1.circle) {
                rightRear.setRotatePower(.5);
            } else {
                rightRear.setRotatePower(0);
            }
            if (gamepad1.dpad_right) {
                rightRear.setDrivePower(.3);
            } else {
                rightRear.setDrivePower(0);
            }

            telemetry.addData("rightFront", "%f", rightFront.getAngle());
            telemetry.addData("leftFront", "%f", leftFront.getAngle());
            telemetry.addData("rightRear", "%f", rightRear.getAngle());
            telemetry.addData("leftRear", "%f", leftRear.getAngle());
            telemetry.addData("Loop", et.toString());
            et.reset();
            telemetry.update();
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        hwinit();
        while (!isStarted()) idle();
        ElapsedTime et = new ElapsedTime();
        et.reset();
        // Stuff to trigger only when a button is first pressed...
        boolean tri = false, sq = false, cross = false, cir = false;
        boolean up = false, down = false, left = false, right = false;

        int state = 0; // 0 -> setting, 1-> trying
        // Setting state information
        double bumpValue = 0.01;
        double curValue = 0.01;
        int which = 0; // 0 = P, 1 = I, 2 = D
        // Trying state information
        double targetAngle = 0.0;
        double error = 0.0;
        double value = 0.0;
        double integral = 0;
        double deriv = 0;
        double delta = 45;
        SwerveModule testMod = rightRear;
        while (opModeIsActive()) {
            if (state == 0) {
                // We're setting values to try
                // left/right change the range of the number adding/subtracting
                // up/down add/sub the number to the current value
                if (gamepad1.dpad_left != left && !left) {
                    bumpValue *= 10;
                }
                if (gamepad1.dpad_right != right && !right) {
                    bumpValue /= 10;
                }
                if (gamepad1.dpad_up != up && !up) {
                    curValue += bumpValue;
                }
                if (gamepad1.dpad_down != down && !down) {
                    curValue -= bumpValue;
                }
                // Triangle: Set the value to the current element selected.
                if (gamepad1.triangle != tri && !tri) {
                    switch (which) {
                        case 0:
                            SwerveModule.servoPid.p = curValue;
                            break;
                        case 1:
                            SwerveModule.servoPid.i = curValue;
                            break;
                        case 2:
                            SwerveModule.servoPid.d = curValue;
                            break;
                        case 3:
                            SwerveModule.servoPid.f = curValue;
                            break;
                    }
                }
                // Square: Move the previous element
                if (gamepad1.square != sq && !sq) {
                    which = (which + 3) % 4;
                }
                // Circle: Move the next element
                if (gamepad1.circle != cir && !cir) {
                    which = (which + 1) % 4;
                }
                // Bumpers change the angle-change to test with
                if (gamepad1.left_bumper && et.milliseconds() > 200) {
                    delta -= 1;
                    et.reset();
                }
                if (gamepad1.right_bumper && et.milliseconds() > 200) {
                    delta += 1;
                    et.reset();
                }
                // Cross: Try the new values (on release so the try state has a chance to run)
                if (gamepad1.cross != cross && cross) {
                    testMod.resetPidData();
                    targetAngle = (testMod.getAngle() + delta + 360) % 360;
                    state = 1;
                    et.reset();
                }
                tri = gamepad1.triangle;
                cir = gamepad1.circle;
                sq = gamepad1.square;
                cross = gamepad1.cross;
                down = gamepad1.dpad_down;
                left = gamepad1.dpad_left;
                right = gamepad1.dpad_right;
                up = gamepad1.dpad_up;
            } else {
                Pair<Double, Double> errVal = testMod.headToDegrees(targetAngle);
                error = errVal.first;
                value = errVal.second;
                if (gamepad1.cross || gamepad1.circle || gamepad1.triangle || gamepad1.square) {
                    // Stop applying power first
                    testMod.setRotatePower(0);
                    // Now, wait for the button to be released before continuing
                    while (gamepad1.cross || gamepad1.circle || gamepad1.triangle || gamepad1.square)
                        ;
                    state = 0;
                }
            }
            telemetry.addData("State", "%s (%1.0f° swing: use bumpers)",
                    state == 0 ? "Setting" : "Trying", delta);
            telemetry.addData((which == 0 && state == 0) ? "[>P<]" : "<[P]>",
                    "%1.1e", SwerveModule.servoPid.p);
            telemetry.addData((which == 1 && state == 0) ? "[>I<]" : "<[I]>",
                    "%1.1e", SwerveModule.servoPid.i);
            telemetry.addData((which == 2 && state == 0) ? "[>D<]" : "<[D]>",
                    "%1.1e", SwerveModule.servoPid.d);
            telemetry.addData((which == 3 && state == 0) ? "[>F<]" : "<[F]>",
                    "%1.1e", SwerveModule.servoPid.f);
            telemetry.addLine(String.format("Bump: %1.4e, Cur: %1.4e", bumpValue, curValue));
            telemetry.addLine();
            // Help text:
            telemetry.addLine("                      ↑Cur+=Bump            △set");
            telemetry.addLine("Bump*10←  →Bump/10\tprev□  ○next");
            telemetry.addLine("                      ↓Cur-=Bump              ╳try it");
            telemetry.addLine(state == 0 ? "" : "Pres any △□◯╳ to stop");
            if (state == 1) {
                double curAngle = testMod.getAngle();
                telemetry.addData("Angles",
                        "%3.1f->%3.1f (%3.1f)", curAngle, targetAngle, error);
                telemetry.addData("Power", value);
                Log.d("PIDF",
                        String.format("%f\tTarget\t%3.1f\tCurrent\t%3.1f\tError\t%3.1f\tPower\t%1.8f",
                                et.milliseconds(), targetAngle, curAngle,
                                error, value));
            }
            telemetry.update();
        }
    }
}
