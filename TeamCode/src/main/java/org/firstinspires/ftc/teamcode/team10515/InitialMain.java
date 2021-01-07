package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.lib.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.lib.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.lib.util.TimeUnits;
import org.firstinspires.ftc.teamcode.team10515.states.FlickerStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.EndGameExtensionStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FeederStoneGripperStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FlywheelStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ForkliftStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.ShooterStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.FoundationStateMachine;
import org.firstinspires.ftc.teamcode.team10515.states.TimedState;
import org.firstinspires.ftc.teamcode.team10515.subsystems.Feeder;

/*
 * This {@code class} acts as the driver-controlled program for FTC team 10515 for the Skystone
 * challenge. By extending {@code SkystoneRobot}, we already have access to all the robot subsystems,
 * so only tele-operated controls need to be defined here.
 *
 * The controls for this robot are:
 *  User 1:
 *      Drive:
 *          Left & Right joysticks     -> Mecanum drive
 *          Left-stick-button          -> Robot speed to default value
 *          Right-stick-button         -> Disable/enable movement
 *          Left-Trigger               -> Decrease robot speed
 *          Right-Trigger              -> Increase robot speed
 *      Forklift:
 *          A-Button (pressed)         -> Lower forklift
 *          Y-Button (pressed)         -> Higher forklift
 *  User 2:
 *      Flywheel Intake:
 *          Left bumper (pressed)      -> Stop Intake
 *          Right bumper (pressed)     -> Start Intake
 *      Shooter:
 *          Left-trigger               -> Stop shoooter
 *          Right-trigger              -> Start shooter
 *          A-button (pressed)         -> Hit ring into launch position (servo)
 *          Y-button (pressed)         -> Toggle shooter
 *      Shooter Speed:
 *          Dpad-right                 -> Shooter speed to 1
 *          Dpad-down                  -> Shooter speed to 2
 *          Dpad-left                  -> Shooter speed to 3
 *          Dpad-up                    -> Shooter speed to 4
 *
 * @see UltimateGoalRobot
 */

@TeleOp(name = "Main Tele-Op", group = "Main")
public class InitialMain extends UltimateGoalRobot {

    public double currentTime = 0; // keep track of current time
    public double previousTime = 0; // keep track of last time A was pressed (Flicker was moved)
    public double flickerInterval = 1; // after 1 second has passed since pressing A, move Flicker back to original position

    public boolean shooterIsOn = false;
    public ShooterStateMachine.State currentShooterSpeed = ShooterStateMachine.State.SPEED1;

    public boolean allowMovement = true;

    @Override
    public void start() {
        Feeder.setManualControlExtension(() -> gamepad2.b ? 0.5d : gamepad2.x ? -0.5d : 0d);
    }

    @Override
    public void loop() {
        super.loop();
        if(allowMovement) { //Make sure driver wants movement
            //Update drive
            setDrivetrainPower(new Pose2d(-gamepad1.left_stick_y, -gamepad1.left_stick_x, new Rotation2d(gamepad1.right_stick_x, false)));
        }

        //-----------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 1

        if(getEnhancedGamepad1().isLeft_stick_button()){
            //Set robot speed to default value
        }

        if(getEnhancedGamepad1().isRight_stick_button()){
            allowMovement = !allowMovement;
        }

        if(getEnhancedGamepad1().getLeft_trigger() > 0){
            //Decrease robot speed
        }

        if(getEnhancedGamepad1().getRight_trigger() > 0){
            //Increase robot speed
        }

        if(getEnhancedGamepad1().isA()){
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.DOWN);
        }
        if(getEnhancedGamepad1().isY()){
            getForkliftSubsystem().getStateMachine().updateState(ForkliftStateMachine.State.UP);
        }

        //-----------------------------------------------------------------------------------------------------------------------------------------
        //Gamepad 2

        //Update servo for ring hitter
        if(getEnhancedGamepad2().isA()){
            getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.HOLD); // move the servo to hit the ring
            previousTime = getRuntime();
        }

        if(currentTime-previousTime >= flickerInterval){ // if the current time minus the last time A was pressed is at least one second,
            getFlickerSubsystem().getStateMachine().updateState(FlickerStateMachine.State.DROP); // move the servo to original position
        }

        //Check for Y button
        if(getEnhancedGamepad2().isY()){
            shooterIsOn = !shooterIsOn;
        }

        //Check for triggers
        if(getEnhancedGamepad2().getRight_trigger() > 0){
            shooterIsOn = true;
        }
        else if(getEnhancedGamepad2().getLeft_trigger() > 0){
            shooterIsOn = false;
        }

        //Check whether to start or stop shooter
        if(shooterIsOn){
            getShooter().getStateMachine().updateState(currentShooterSpeed);
        }
        else{
            getShooter().getStateMachine().updateState(ShooterStateMachine.State.IDLE);
        }

        //Check for bumpers
        if(getEnhancedGamepad2().isLeft_bumper()){
            getFlywheels().getStateMachine().updateState(FlywheelStateMachine.State.IDLE);
        }
        else if(getEnhancedGamepad2().isRight_bumper()) {
            getFlywheels().getStateMachine().updateState(FlywheelStateMachine.State.INTAKE);
        }

        //Check for DPad
        if(getEnhancedGamepad2().isDpad_right()){
            currentShooterSpeed = ShooterStateMachine.State.SPEED1;
            //testing
        }
        else if(getEnhancedGamepad2().isDpad_down()){
            currentShooterSpeed = ShooterStateMachine.State.SPEED2;
        }
        else if(getEnhancedGamepad2().isDpad_left()){
            currentShooterSpeed = ShooterStateMachine.State.SPEED3;
        }
        else if(getEnhancedGamepad2().isDpad_up()){
            currentShooterSpeed = ShooterStateMachine.State.SPEED4;
        }


        //--------------------------------------------------------------------------------------------------------------------------------
        // last year

        //Update flywheel intake
        if(getEnhancedGamepad2().getRight_trigger()>0) {
            getFlywheels().getStateMachine().updateState(FlywheelStateMachine.State.INTAKE);
        } else if(getEnhancedGamepad2().getLeft_trigger() > 0) {
            getFlywheels().getStateMachine().updateState(FlywheelStateMachine.State.OUTTAKE);
        } else if(gamepad2.back) {
            getFlywheels().getStateMachine().updateState(FlywheelStateMachine.State.IDLE);
        }

        //Update Stack tracker
        if(getEnhancedGamepad2().isDpadRightJustPressed()) {
            getStackTracker().addStoneToStack();
        } else if(getEnhancedGamepad2().isDpadLeftJustPressed()) {
            getStackTracker().removeStoneFromStack();
        } else if(gamepad1.left_trigger > 0.05d) {
            getStackTracker().resetStack();
        }

        //Update feeder
        if(getEnhancedGamepad2().isyJustPressed()) {
            getFeeder().extend();
        } else if(getEnhancedGamepad2().isaJustPressed()) {
            getFeeder().retract();
        }

        //Toggle virtual four-bar
        if(getEnhancedGamepad2().isLeftBumperJustPressed()) {
            getFeeder().toggleVirtualFourBar();
        }

        //Check to release grip of stone for stacking
        if(getEnhancedGamepad2().isRight_bumper() /*&& getStackTracker().getExtensionHeight() == Feeder.getSetpoint()*/) {
            Feeder.getFeederStoneGripperStateMachine().updateState(FeederStoneGripperStateMachine.State.NO_GRIP);
            //  getFeeder().toggleStoneGripper();
        }

        if(getEnhancedGamepad2().isStart()){
            Feeder.getFeederStoneGripperStateMachine().updateState(FeederStoneGripperStateMachine.State.GRIP);
        }

        if(getEnhancedGamepad1().isB()){
            getFeeder().setDeliveryMode(true);
        }else if(getEnhancedGamepad1().isX()){
            getFeeder().setDeliveryMode(false);
        }

        //Toggle end game extension blocker to extend slides
        if(getEnhancedGamepad1().isBack()) {
            getEndGameExtensionSubsystem().getStateMachine().updateState(EndGameExtensionStateMachine.State.RELEASE_SLIDES);
        }

        if(getEnhancedGamepad2().isDpadDownJustPressed()) {
            getFoundationSubsystem().getStateMachine().updateState(FoundationStateMachine.State.GRAB);
        } else if(getEnhancedGamepad2().isDpadUpJustPressed()) {
            getFoundationSubsystem().getStateMachine().updateState(FoundationStateMachine.State.INIT);
        }

        if(getEnhancedGamepad1().isA()) {
            Feeder.getFlickerStateMachine().updateState(FlickerStateMachine.State.DROP);
        } else if(getEnhancedGamepad1().isY()) {
            Feeder.getFlickerStateMachine().updateState(FlickerStateMachine.State.HOLD);
        }


        telemetry.addLine("Stones stacked: " + getStackTracker());
        telemetry.addLine("Stacked Height: " + getStackTracker().getExtensionHeight());
        telemetry.addLine("Extension Setpoint: " + Feeder.getSetpoint());
        telemetry.addLine("Extension Desired Setpoint: " + Feeder.getDesiredSetpoint());
        telemetry.addLine("Extension Height: " + getFeeder().getLeftExtension().getPosition());
        telemetry.addLine("Extension State: " + Feeder.getFeederExtensionStateMachine().getState().getName());
        telemetry.addLine("Left Extension Power: " + getFeeder().getLeftExtension().getLastPower());
        telemetry.addLine("Right Extension Power: " + getFeeder().getRightExtension().getLastPower());
        telemetry.addLine("V4B State: " + Feeder.getVirtualFourBarStateMachine().getState().getName());
        telemetry.addLine("Time seen stone: " + getFeeder().getTimeProfilerStoneDetection().getDeltaTime(TimeUnits.SECONDS, false));
        telemetry.addLine("Stone distance: " + getFeeder().getStoneDetector().getDistance(DistanceUnit.INCH));
        telemetry.addLine("Feeder Extension Constants: " + Feeder.getExtendControlConstants());
        telemetry.addLine("Extension close to setpoint: " + getFeeder().closeToSetpoint(1 / 4d));
        telemetry.addLine("Extension Profile: " + (Feeder.getExtensionProfile() != null));
        if(Feeder.getExtensionProfile() != null) {
            telemetry.addLine("" + Feeder.getExtensionProfile().getPosition());
        }

        currentTime = getRuntime();

    }
}
