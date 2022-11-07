package org.firstinspires.ftc.teamcode.robots.taubot;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.auto;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.robot;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.startingPosition;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.alliance;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.active;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.debugTelemetryEnabled;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.visionProviderFinalized;
import static org.firstinspires.ftc.teamcode.robots.taubot.PowerPlay_6832.visionProviderIndex;
import static org.firstinspires.ftc.teamcode.robots.taubot.util.Utils.notJoystickDeadZone;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.robots.taubot.util.Constants;
import org.firstinspires.ftc.teamcode.robots.taubot.util.StickyGamepad;
import org.firstinspires.ftc.teamcode.robots.taubot.vision.VisionProviders;


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

        if (notJoystickDeadZone(gamepad1.right_stick_x)) robot.crane.adjustTurretAngle(-gamepad1.right_stick_x);
        if (notJoystickDeadZone(gamepad1.right_stick_y)) robot.crane.adjustShoulder(-0.7*gamepad1.right_stick_y);

        if (gamepad1.right_trigger>.05) robot.crane.adjustExtend(gamepad1.right_trigger);
        if (gamepad1.left_trigger>.05) robot.crane.adjustExtend(-gamepad1.left_trigger);

        if(stickyGamepad1.a) {
            robot.crane.pickupSequence();
        }

        if(stickyGamepad1.b){
            robot.crane.dropSequence();
        }

        //manual override of drivetrain
        if (notJoystickDeadZone(gamepad1.left_stick_y) || notJoystickDeadZone(gamepad1.left_stick_x))
            robot.driveTrain.ManualArcadeDrive(-gamepad1.left_stick_y,  gamepad1.left_stick_x);
        else {
            robot.driveTrain.ManualDriveOff();
        }

        /*
        if( stickyGamepad1.a){
            robot.updateFieldTargetPose(0, -1);
        }
        if( stickyGamepad1.y){
            robot.updateFieldTargetPose(0, 1);
        }
        if( stickyGamepad1.x){
            robot.updateFieldTargetPose(-1, 0);
        }
        if( stickyGamepad1.b){
            robot.updateFieldTargetPose(1, 0);
        }

        if(stickyGamepad1.right_bumper){
            robot.field.goToHighPole(robot);
        }
        else if (stickyGamepad1.left_bumper) {
            robot.field.goToStack(robot);
        }
        //if (notJoystickDeadZone(gamepad1.right_stick_x))
            //pwrRot = pwrDamper * .75 * gamepad1.right_stick_x;


        if (nearZero(pwrFwd) && nearZero(pwrRot) && robot.isNavigating) {
        } else {
            robot.isNavigating = false; // take control back from any auton navigation if any joystick input is running
            robot.autonTurnInitialized = false;
            robot.driveMixerDiffSteer(pwrFwd * pwrDamper, pwrRot);
        }
        */
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

