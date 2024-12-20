package org.firstinspires.ftc.teamcode.BBcode.Tuning;

import static org.firstinspires.ftc.robotcontroller.external.samples.RobotHardware.MID_SERVO;
//import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BBcode.TelemetryHelper;
import org.firstinspires.ftc.teamcode.tuning.Drawing;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "MotorTuner")
@Config
public class MotorTuner extends LinearOpMode {
    enum InputState {
        None,
        A,
        B,
        X,
        Y,
        LT,
        RT,
        LB,
        RB,
        DPadUp,
        DPadDown,
        DPadLeft,
        DPadRight,
        LeftStick,
        RightStick
    }
    FtcDashboard dashboard;
    public static int selectedIndex = 0;
    public static double VelocityMultiplier = 10000;
    public static double velocity = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = FtcDashboard.getInstance();
        boolean foo = dashboard.isEnabled();
        //telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry = dashboard.getTelemetry();
        // Get a list of all the servo names in the robot configuration
        List<String> motorNamesList = new ArrayList<String>();
        for (DcMotor motor : hardwareMap.dcMotor) {
            //Devices can have multiple names, so we concatenate them with a colon
            String concatenatedNames = String.join(":", hardwareMap.getNamesOf(motor));
            motorNamesList.add(concatenatedNames);
        }
        String[] motorNames = motorNamesList.toArray(new String[0]);
        InputState inputState = InputState.None;
        //int selectedIndex = 0;
        DcMotorEx dcMotorEx = null;
        boolean yPressed = false;
        boolean aPressed = false;
        boolean bPressed = false;
        boolean xPressed = false;
        boolean dpadUpPressed = false;
        boolean dpadDownPressed = false;
        boolean dpadLeftPressed = false;
        boolean dpadRightPressed = false;
        boolean leftTriggerPressed = false;
        boolean rightTriggerPressed = false;
        int motorPosition = 0;
        double increment = 0.1; // Start with tenths


        //TelemetryHelper telemetryHelper = new TelemetryHelper(this);
        //telemetryHelper.initGamepadTelemetry(gamepad1);

        // Allow the user to select a servo before start is pressed
        while (!isStarted()) {

            if (gamepad1.left_trigger > 0 && !leftTriggerPressed) {
                selectedIndex = (selectedIndex - 1 + motorNames.length) % motorNames.length;
                leftTriggerPressed = true;
            } else if (gamepad1.left_trigger == 0) {
                leftTriggerPressed = false;
            }

            if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
                selectedIndex = (selectedIndex + 1) % motorNames.length;
                rightTriggerPressed = true;
            } else if (gamepad1.right_trigger == 0) {
                rightTriggerPressed = false;
            }
            if (gamepad1.x && !xPressed) {
                xPressed = true;
            } else if (!gamepad1.x) {
                xPressed = false;
            }
            if (gamepad1.b && !bPressed) {
                bPressed = true;
            } else if (!gamepad1.b) {
                bPressed = false;
            }
            if (gamepad1.a && !aPressed) {
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }
            if (gamepad1.y && !yPressed) {
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            telemetry.addData("Instructions", "Use y/a to select a motor.");
            telemetry.addData("Selected motor", motorNames[selectedIndex]);
            telemetry.update();
        }

        dcMotorEx = getDcMotorEx(motorNames[selectedIndex]);
        waitForStart();

        // Allow the user to control the servo during the main loop
        while (opModeIsActive()) {
            if (gamepad1.left_trigger > 0 && !leftTriggerPressed) {
                int newIndex = (selectedIndex - 1 + motorNames.length) % motorNames.length;
                if (newIndex != selectedIndex) {
                    selectedIndex = newIndex;
                    dcMotorEx = getDcMotorEx(motorNames[selectedIndex]);
                }
                leftTriggerPressed = true;
            } else if (gamepad1.left_trigger == 0) {
                leftTriggerPressed = false;
            }

            if (gamepad1.right_trigger > 0 && !rightTriggerPressed) {
                int newindex = (selectedIndex + 1) % motorNames.length;
                if (newindex != selectedIndex) {
                    selectedIndex = newindex;
                    dcMotorEx = getDcMotorEx(motorNames[selectedIndex]);
                }
                rightTriggerPressed = true;
            } else if (gamepad1.right_trigger == 0) {
                rightTriggerPressed = false;
            }
            if (gamepad1.x && !xPressed) {
                xPressed = true;
            } else if (!gamepad1.x) {
                xPressed = false;
            }
            if (gamepad1.b && !bPressed) {
                bPressed = true;
            } else if (!gamepad1.b) {
                bPressed = false;
            }
            if (gamepad1.a && !aPressed) {
                aPressed = true;
            } else if (!gamepad1.a) {
                aPressed = false;
            }
            if (gamepad1.y && !yPressed) {
                yPressed = true;
            } else if (!gamepad1.y) {
                yPressed = false;
            }

            if (gamepad1.dpad_up && !dpadUpPressed) {
                dpadUpPressed = true;
                velocity += increment;
            } else if (!gamepad1.dpad_up) {
                dpadUpPressed = false;
            }
            if (gamepad1.dpad_down && !dpadDownPressed) {
                dpadDownPressed = true;
                velocity -= increment;

            } else if (!gamepad1.dpad_down) {
                dpadDownPressed = false;
            }
            if (gamepad1.dpad_right && !dpadRightPressed) {
                increment = Math.max(increment / 10, 0.001); // Decrease order of magnitude
                dpadRightPressed = true;
            } else if (!gamepad1.dpad_right) {
                dpadRightPressed = false;
            }

            if (gamepad1.dpad_left && !dpadLeftPressed) {
                increment = Math.min(increment * 10, 0.1); // Increase order of magnitude
                dpadLeftPressed = true;
            } else if (!gamepad1.dpad_left) {
                dpadLeftPressed = false;
            }

            //dcMotorEx.setVelocity(gamepad1.left_stick_y * VelocityMultiplier);
            dcMotorEx.setVelocity(velocity);
            telemetry.addData("Instructions", "Use D-pad up/down to change by " + increment + ": Use buttons X=0, Y=0.5, B=1.0: Use left/right to change order of magnitude.");
            telemetry.addData("Selected Motor", motorNames[selectedIndex]);
            telemetry.addData("Motor Position", motorPosition);
            telemetry.update();
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("pos", dcMotorEx.getCurrentPosition());
            packet.put("power", dcMotorEx.getPower() * 400);
            packet.put("vel", dcMotorEx.getVelocity(AngleUnit.DEGREES));
            packet.put("velocity", velocity);
            packet.fieldOverlay().setStroke("#3F51B5");
            //Drawing.drawRobot(packet.fieldOverlay(), drive.pose);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);
        }
    }

    @NonNull
    private DcMotorEx getDcMotorEx(String motorNames) {
        DcMotorEx dcMotorEx;
        TelemetryHelper telemetryHelper = new TelemetryHelper(this);
        dcMotorEx = hardwareMap.get(DcMotorEx.class, motorNames);
        dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //telemetryHelper.initMotorTelemetry(dcMotorEx, motorNames);
        return dcMotorEx;
    }
}
