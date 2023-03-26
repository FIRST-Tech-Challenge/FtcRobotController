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

import org.firstinspires.ftc.teamcode.robots.taubot.subsystem.Crane;
import org.firstinspires.ftc.teamcode.robots.taubot.subsystem.Robot;
import org.firstinspires.ftc.teamcode.robots.taubot.subsystem.UnderArm;
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

    public static double TURRET_DEADZONE = 0.03;

    void joystickDrive() {
        if(stickyGamepad2.dpad_right)
            robot.transferAdvance();

        if(gamepad1.dpad_up){
            robot.crane.articulate(Crane.Articulation.home);
        }

        if(gamepad1.left_bumper || gamepad2.left_bumper){
            robot.driveTrain.adjustChassisLength(-1);
        }

        if(gamepad1.right_bumper || gamepad2.right_bumper){
            robot.driveTrain.adjustChassisLength(1);
        }

        if(stickyGamepad1.start){
            robot.driveTrain.toggleExtension();
            //if (robot.driveTrain.getChariotDeployed())
                //robot.crane.articulate(Crane.Articulation.manualDrive); //allow the crane and turret to return to manual mode
        }

        if (stickyGamepad1.guide || stickyGamepad2.guide){ //this is the Logitech button
                robot.crane.resetArticulations();
                robot.underarm.resetArticulations();
                robot.resetArticulations();
                robot.articulate((Robot.Articulation.TRANSFER));
        }

        if(stickyGamepad1.a) {
            //maintain the ability to pickup with Crane
            robot.crane.pickupSequence();
        }
        if(stickyGamepad1.b){
            robot.articulate(Robot.Articulation.DROP);
        }

        if(stickyGamepad1.x){
            robot.field.incTarget();
            robot.crane.updateScoringPattern();
        }

        if(stickyGamepad1.y){
            robot.field.decTarget();
            robot.crane.updateScoringPattern();
        }

        if(stickyGamepad1.dpad_right){
            robot.field.incScoringPattern();
            robot.crane.updateScoringPattern();
        }

        if(stickyGamepad1.dpad_left){
            robot.field.decScoringPattern();
            robot.crane.updateScoringPattern();
        }

        if(notJoystickDeadZone(gamepad1.left_stick_x)){
            robot.driveTrain.maxTuck();
        }

        if(!gamepad1.dpad_down) {
            if (notJoystickDeadZone(gamepad1.right_stick_x)){
                robot.crane.adjustY(-0.4*gamepad1.right_stick_x);
            }

            if (notJoystickDeadZone(gamepad1.right_stick_y)) {
                robot.crane.adjustX(-0.4*gamepad1.right_stick_y);
            }

            if (gamepad1.right_trigger>.05) robot.crane.adjustZ(0.7*gamepad1.right_trigger);
            if (gamepad1.left_trigger>.05) robot.crane.adjustZ(-0.7*gamepad1.left_trigger);

            //manual override of drivetrain
            if (notJoystickDeadZone(gamepad1.left_stick_y) || notJoystickDeadZone(gamepad1.left_stick_x)) {
                robot.driveTrain.ManualArcadeDrive(-0.7 * gamepad1.left_stick_y, 0.7 * gamepad1.left_stick_x);
                robot.driverIsDriving();
            }
            else {
                robot.driveTrain.ManualDriveOff();
                robot.driverNotDriving();
            }
        }else{
            if (notJoystickDeadZone(gamepad1.right_stick_x)){
                robot.crane.adjustY(-gamepad1.right_stick_x);
            }

            if (notJoystickDeadZone(gamepad1.right_stick_y)) {
                robot.crane.adjustX(-gamepad1.right_stick_y);
            }

            if (gamepad1.right_trigger>.05) robot.crane.adjustZ(gamepad1.right_trigger);
            if (gamepad1.left_trigger>.05) robot.crane.adjustZ(-gamepad1.left_trigger);

            //manual override of drivetrain
            if (notJoystickDeadZone(gamepad1.left_stick_y) || notJoystickDeadZone(gamepad1.left_stick_x)) {
                robot.driveTrain.ManualArcadeDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x);
                robot.driverIsDriving();
            }
            else {
                robot.driveTrain.ManualDriveOff();
                robot.driverNotDriving();
            }
        }
    }

    void joystickDriveDemoMode(){
        if(gamepad1.dpad_up){
            robot.crane.articulate(Crane.Articulation.home);
        }

        if(gamepad1.left_bumper){
            robot.driveTrain.adjustChassisLength(-0.7);
        }

        if(gamepad1.right_bumper){
            robot.driveTrain.adjustChassisLength(0.7);
        }

        if(stickyGamepad1.start){
            robot.driveTrain.toggleExtension();
            //if (robot.driveTrain.getChariotDeployed())
                //robot.crane.articulate(Crane.Articulation.manual); //allow the crane and turret to return to manual mode
        }

        if (stickyGamepad1.guide){ //this is the Logitech button
            robot.articulate((Robot.Articulation.TRANSFER));

        }

        if(stickyGamepad1.a) {
            //maintain the ability to pickup with Crane
            robot.crane.pickupSequence();
        }

        if(stickyGamepad1.b){
            robot.crane.dropSequence();
            robot.field.incTarget();
            robot.crane.updateScoringPattern();
        }

        if(stickyGamepad1.x){
            robot.field.incTarget();
            robot.crane.updateScoringPattern();
        }

        if(stickyGamepad1.y){
            robot.field.decTarget();
            robot.crane.updateScoringPattern();
        }

        if(stickyGamepad1.dpad_right){
            robot.field.incScoringPattern();
            robot.crane.updateScoringPattern();
        }

        if(stickyGamepad1.dpad_left){
            robot.field.decScoringPattern();
            robot.crane.updateScoringPattern();
        }

        if(notJoystickDeadZone(gamepad1.left_stick_x)){
            robot.driveTrain.maxTuck();
        }

        if(!gamepad1.dpad_down) {
            if (notJoystickDeadZone(gamepad1.right_stick_x)){
                robot.crane.adjustY(-0.4*gamepad1.right_stick_x);
            }

            if (notJoystickDeadZone(gamepad1.right_stick_y)) {
                robot.crane.adjustX(-0.4*gamepad1.right_stick_y);
            }

            if (gamepad1.right_trigger>.05) robot.crane.adjustZ(0.7*gamepad1.right_trigger);
            if (gamepad1.left_trigger>.05) robot.crane.adjustZ(-0.7*gamepad1.left_trigger);

            //manual override of drivetrain
            robot.driveTrain.gridMove(-0.7 * gamepad1.left_stick_y, -0.7 * gamepad1.left_stick_x);
            if (notJoystickDeadZone(gamepad1.left_stick_y) || notJoystickDeadZone(gamepad1.left_stick_x)) {
                robot.driverIsDriving();
            }
            else {
                robot.driveTrain.ManualDriveOff();
                robot.driverNotDriving();
            }
        }else{
            if (notJoystickDeadZone(gamepad1.right_stick_x)){
                robot.crane.adjustY(-gamepad1.right_stick_x);
            }

            if (notJoystickDeadZone(gamepad1.right_stick_y)) {
                robot.crane.adjustX(-gamepad1.right_stick_y);
            }

            if (gamepad1.right_trigger>.05) robot.crane.adjustZ(gamepad1.right_trigger);
            if (gamepad1.left_trigger>.05) robot.crane.adjustZ(-gamepad1.left_trigger);

            //manual override of drivetrain
            robot.driveTrain.gridMove(-gamepad1.left_stick_y, -gamepad1.left_stick_x);
            if (notJoystickDeadZone(gamepad1.left_stick_y) || notJoystickDeadZone(gamepad1.left_stick_x)) {
                robot.driverIsDriving();
            }
            else {
                robot.driveTrain.ManualDriveOff();
                robot.driverNotDriving();
            }
        }
    }

    void joystickDrivePregameMode() {
        // drive joysticks
        robot.driveTrain.ManualTankDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        if(stickyGamepad1.y) {
            robot.crane.toggleGripper();
        }

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
            alliance = Constants.Alliance.BLUE;        }
        if(stickyGamepad1.b || stickyGamepad2.b) {
            alliance = Constants.Alliance.RED;
        }

        if(stickyGamepad1.dpad_left || stickyGamepad2.dpad_left)startingPosition = Constants.Position.START_LEFT;
        if(stickyGamepad1.dpad_right || stickyGamepad2.dpad_right)startingPosition = Constants.Position.START_RIGHT;

        if(previousStartingPosition != startingPosition) {
            robot.driveTrain.setPoseEstimate(startingPosition.getPose());
            auto.build(startingPosition);
        }
        robot.driveTrain.cachePositionForNextRun(startingPosition.getPose());
        if(stickyGamepad1.dpad_up || stickyGamepad2.dpad_up)
            debugTelemetryEnabled = !debugTelemetryEnabled;

        if(stickyGamepad1.dpad_down){
            robot.articulate(Robot.Articulation.CALIBRATE);
        }

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

    public void UnderarmControls(){

        if(stickyGamepad2.a){
            //robot.articulate(Robot.Articulation.PICKUP);
            robot.underarm.resetArticulations();
            robot.underarm.articulate(UnderArm.Articulation.substationHover);
        }

        if (stickyGamepad2.b) {
            //underarm driver might have re-positioned the hover, so save the values for next time
            robot.underarm.SaveHoverPositions();
            robot.underarm.resetArticulations();
            robot.underarm.articulate(UnderArm.Articulation.substationPickup);
        }

        if (stickyGamepad2.x)// cancel transfer position reset and drop cone
        {
            robot.transferAdvance();
        }

        if(stickyGamepad2.y){
            robot.underarm.toggleLasso();
        }

        if(stickyGamepad2.start){
            robot.driveTrain.toggleExtension();
        }

        if(stickyGamepad2.dpad_up){
            robot.underarm.articulate(UnderArm.Articulation.home);
        }

        if(notJoystickDeadZone(gamepad2.left_stick_y)){
            robot.underarm.adjustShoulder(gamepad2.left_stick_y);
        }

        if(robot.getArticulation().equals(Robot.Articulation.TRANSFER) && (notJoystickDeadZone(gamepad2.left_trigger) || notJoystickDeadZone(gamepad2.right_trigger)))
            robot.articulate(Robot.Articulation.CANCEL_TRANSFER);

        if(notJoystickDeadZone(gamepad2.right_stick_y)){
            robot.underarm.adjustElbow(-gamepad2.right_stick_y);
        }

        if(notJoystickDeadZone(gamepad2.right_stick_x)){
            robot.underarm.adjustWrist(-gamepad2.right_stick_x);
        }

        if(notJoystickDeadZone(gamepad2.left_stick_x)){
            robot.underarm.adjustTurret(gamepad2.left_stick_x);
        }

        if (gamepad1.right_trigger>.05) robot.underarm.adjustZ(gamepad1.right_trigger);
        if (gamepad1.left_trigger>.05) robot.underarm.adjustZ(-gamepad1.left_trigger);

    }

    public void UnderarmTesting(){
        if(Math.abs(gamepad1.left_stick_y) > 0.05){
            robot.underarm.adjustShoulder(gamepad1.left_stick_y);
        }
        if(Math.abs(gamepad1.right_stick_y) > 0.05){
            robot.underarm.adjustElbow(gamepad1.right_stick_y);
        }
        if(Math.abs(gamepad1.left_stick_x) > 0.05){
            robot.underarm.adjustTurret(gamepad1.left_stick_x);
        }


    }
}

