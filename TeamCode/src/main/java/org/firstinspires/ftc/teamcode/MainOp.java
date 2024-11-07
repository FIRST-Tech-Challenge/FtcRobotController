package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Settings.ControllerProfile;

/** @noinspection unused */
@TeleOp(name = "MainOp", group = "TeleOp")
public class MainOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Show profile selection menu before initializing robot
        ControllerProfile selectedProfile = Settings.DEFAULT_PROFILE;
        boolean menuActive = true;

        while (!isStarted() && !isStopRequested() && menuActive) {
            String[] options = new String[Settings.AVAILABLE_PROFILES.length + 1];
            for (int i = 0; i < Settings.AVAILABLE_PROFILES.length; i++) {
                options[i] = Settings.AVAILABLE_PROFILES[i].name;
            }
            options[options.length - 1] = "Confirm";

            telemetry.addLine("Select Controller Profile:");
            telemetry.addData("Current Selection", selectedProfile.name);

            // Handle profile selection using gamepad
            if (gamepad1.dpad_up) {
                int currentIndex = getCurrentProfileIndex(selectedProfile);
                if (currentIndex > 0) {
                    selectedProfile = Settings.AVAILABLE_PROFILES[currentIndex - 1];
                }
                sleep(250); // Debounce
            } else if (gamepad1.dpad_down) {
                int currentIndex = getCurrentProfileIndex(selectedProfile);
                if (currentIndex < Settings.AVAILABLE_PROFILES.length - 1) {
                    selectedProfile = Settings.AVAILABLE_PROFILES[currentIndex + 1];
                }
                sleep(250); // Debounce
            } else if (gamepad1.a) {
                menuActive = false;
            }

            telemetry.update();
        }

        // Initialize robot with selected profile
        BaseRobot baseRobot = new BaseRobot(hardwareMap, gamepad1, gamepad2, this, telemetry, selectedProfile);
        waitForStart();

        while (opModeIsActive()) {
            baseRobot.driveGamepads();

            if (Settings.Deploy.ODOMETRY) {
                baseRobot.odometry.update();
            }
        }
        baseRobot.shutDown();
    }

    private int getCurrentProfileIndex(ControllerProfile profile) {
        for (int i = 0; i < Settings.AVAILABLE_PROFILES.length; i++) {
            if (Settings.AVAILABLE_PROFILES[i].name.equals(profile.name)) {
                return i;
            }
        }
        return 0;
    }

}
