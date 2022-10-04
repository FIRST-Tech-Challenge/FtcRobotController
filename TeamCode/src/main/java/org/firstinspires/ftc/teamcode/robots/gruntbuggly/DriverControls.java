package org.firstinspires.ftc.teamcode.robots.gruntbuggly;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.PowerPlay_6832.auto;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.PowerPlay_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.PowerPlay_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.PowerPlay_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.PowerPlay_6832.active;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.PowerPlay_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.PowerPlay_6832.visionProviderFinalized;
import static org.firstinspires.ftc.teamcode.robots.gruntbuggly.PowerPlay_6832.visionProviderIndex;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.robots.gruntbuggly.util.Constants;
import org.firstinspires.ftc.teamcode.robots.gruntbuggly.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.gruntbuggly.vision.VisionProviders;


public class DriverControls {
    // gamepads
    Gamepad gamepad1, gamepad2;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    DriverControls(Gamepad pad1,Gamepad pad2) {
        gamepad1 = pad1;
        gamepad2 = pad2;
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);
    }

    public void init_loop(){
        updateStickyGamepads();
        handleStateSwitch();
        handlePregameControls();
        handleVisionProviderSwitch();
        joystickDrivePregameMode();
    }

    public void updateStickyGamepads(){
        stickyGamepad1.update();
        stickyGamepad2.update();
    }

    void joystickDrive() {
        robot.driveTrain.ManualTankDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
        robot.crane.adjustShoulderAngle(gamepad1.right_trigger);
    }

    void joystickDrivePregameMode() {
        // drive joysticks
        robot.driveTrain.ManualTankDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        // turret controls

    }

    public void handleStateSwitch() {
        if (!active) {
            if (stickyGamepad1.left_bumper || stickyGamepad2.left_bumper)
                PowerPlay_6832.gameStateIndex -= 1;
            if (stickyGamepad1.right_bumper || stickyGamepad2.right_bumper)
                PowerPlay_6832.gameStateIndex += 1;

            if(PowerPlay_6832.gameStateIndex < 0)
                PowerPlay_6832.gameStateIndex = PowerPlay_6832.GameState.getNumGameStates() - 1;
            PowerPlay_6832.gameStateIndex %= PowerPlay_6832.GameState.getNumGameStates();
            PowerPlay_6832.gameState = PowerPlay_6832.GameState.getGameState(PowerPlay_6832.gameStateIndex);
        }

        if (stickyGamepad1.back || stickyGamepad2.back)
            active = !active;
    }

    void handlePregameControls() {
        Constants.Position previousStartingPosition = startingPosition;
        if(stickyGamepad1.x || stickyGamepad2.x) {
            startingPosition = Constants.Position.START_LEFT;
            alliance.Toggle();
        }
        if(stickyGamepad1.b || stickyGamepad2.b) {
            alliance = Constants.Alliance.RED;
        }
        if(previousStartingPosition != startingPosition) {
            //todo these lines need to be enabled once we build the drivetrain and auton routines for powerplay
            //robot.driveTrain.setPoseEstimate(startingPosition.getPose());
            //auto.build(startingPosition);
        }

        if(stickyGamepad1.dpad_up || stickyGamepad2.dpad_up)
            debugTelemetryEnabled = !debugTelemetryEnabled;
        /*
        if(stickyGamepad1.dpad_down || stickyGamepad2.dpad_down)
            if (robot.crane.shoulderInitialized)
                robot.articulate(Robot.Articulation.START_DOWN); //stow crane to the starting position
            else
                robot.crane.configureShoulder(); //setup the shoulder - do this only when the
        if(stickyGamepad1.left_trigger || stickyGamepad2.left_trigger)
            numericalDashboardEnabled = !numericalDashboardEnabled;
        if(stickyGamepad1.right_trigger || stickyGamepad2.right_trigger)
            antiTippingEnabled = !antiTippingEnabled;
        if(stickyGamepad1.right_stick_button || stickyGamepad2.right_stick_button)
            smoothingEnabled = !smoothingEnabled;
        if(stickyGamepad1.left_stick_button || stickyGamepad2.left_stick_button)
            robot.crane.articulate(Crane.Articulation.TEST_INIT);

 */
    }

    public void handleVisionProviderSwitch() {
        if(!active) {
            if(!visionProviderFinalized) {
                if (stickyGamepad1.dpad_left || stickyGamepad2.dpad_left) {
                    visionProviderIndex = (visionProviderIndex + 1) % VisionProviders.VISION_PROVIDERS.length; // switch vision provider
                    auto.createVisionProvider(visionProviderIndex);
                }
                if (stickyGamepad1.dpad_up || stickyGamepad2.dpad_up) {
                    auto.visionProvider.initializeVision(hardwareMap); // this is blocking
                    visionProviderFinalized = true;
                }
            } else if (stickyGamepad1.dpad_up || stickyGamepad2.dpad_up) {
                auto.visionProvider.shutdownVision(); // also blocking, but should be very quick
                visionProviderFinalized = false;
            }
        }
        else if((stickyGamepad1.dpad_right || stickyGamepad2.dpad_right) && visionProviderFinalized)
        {
            auto.visionProvider.saveDashboardImage();
        }
        if(visionProviderFinalized)
            auto.visionProvider.update();
    }
}

