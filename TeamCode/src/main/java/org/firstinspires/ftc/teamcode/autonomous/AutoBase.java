package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.systems.DynamicInput;
import org.firstinspires.ftc.teamcode.BaseRobot;
import org.firstinspires.ftc.teamcode.MainAuto;
import org.firstinspires.ftc.teamcode.systems.ShutdownManager;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Main Autonomous", group = "Autonomous")
public abstract class AutoBase extends LinearOpMode {
    protected String color;
    protected String position;
    private final String defaultColor;
    private final String defaultPosition;

    protected AutoBase(String defaultColor, String defaultPosition) {
        this.defaultColor = defaultColor;
        this.defaultPosition = defaultPosition;
        this.color = defaultColor;
        this.position = defaultPosition;
    }

    @Override
    public void runOpMode() {
//        // Show GUI menu before initializing robot
//        gamepad1.type(Gamepad.LED_PATTERN_SOLID);
//        boolean menuActive = true;
//
//        while (!isStarted() && !isStopRequested() && menuActive) {
//            String[] options = { "Red Left", "Red Right", "Blue Left", "Blue Right", "Confirm" };
//            String message = "Select Auto Configuration:";
//
//            gamepad1.runLedEffect(options);
//            int selection = gamepad1(message, options);
//
//            if (selection >= 0 && selection < 4) {
//                // Parse selection
//                color = selection < 2 ? "red" : "blue";
//                position = selection % 2 == 0 ? "left" : "right";
//            } else if (selection == 4) {
//                menuActive = false;
//            }
//
//            telemetry.addData("Selected Configuration", color + " " + position);
//            telemetry.update();
//        }

        // Initialize robot after selection is complete
        BaseRobot baseRobot = new BaseRobot(hardwareMap, gamepad1, gamepad2, this, telemetry);
        MainAuto auto = new MainAuto(baseRobot, color);
        ShutdownManager shutdownManager = new ShutdownManager(this, baseRobot, auto);

        shutdownManager.scheduleShutdownCheck();

        try {
            if (opModeIsActive()) {
                auto.run(color + " " + position);
            }
        } catch (RuntimeException e) {
            // Shutdown handled by ShutdownManager
        }
    }
}