package org.firstinspires.ftc.teamcode.team;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.lib.drivers.RevMotor;
import org.firstinspires.ftc.teamcode.lib.util.TimeProfiler;
import org.firstinspires.ftc.teamcode.team.auto.PPBaseFeeder;

import java.util.Arrays;

/**
 * Motor naming convention:
 *     Drivetrain
 *         Front Left Wheel -> LF
 *         Back Left Wheel   -> LR
 *         Front Right Wheel -> RF
 *         Back Right Wheel  -> RR
 *      Elevator
 *         Left Motor -> Elev Left
 *         Right Motor -> Elev Right
 *     Arm
 *         Arm Servo  -> Arm
 *      Claw
 *          Gripper -> Claw
 * Misc. sensors naming convention:

 */
public abstract class PPTeleopRobotFeeder extends Robot {
    private TimeProfiler matchRuntime;
    protected PPBaseFeeder drive;
    private static RevBlinkinLedDriver lights;

    @Override
    public void init() {
        super.init();
        setMatchRuntime(new TimeProfiler(false));
        setLights((hardwareMap.get(RevBlinkinLedDriver.class, "blinkin")));
    }

    @Override
    public void init_loop() {
        super.init_loop();
    }

    @Override
    public void start() {
        super.start();
        drive.getExpansionHubs().start();
        drive.robot.getDrive().start();
        Arrays.stream(getMotors()).forEach(RevMotor::resetEncoder);
        drive.robot.getFeeder().start();
        getMatchRuntime().start();
//        getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.VIOLET);
    }

    @Override
    public void loop() {
        super.loop();
        drive.getExpansionHubs().update(getDt());
        drive.update();
        drive.robot.getFeeder().update(getDt());
//        if(drive.robot.getFeeder().getFeederConeGripperStateMachine().hasReachedStateGoal(FeederConeGripperStateMachine.State.CLOSE)) {
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
//        } else if(drive.robot.getStackTracker().getPoleTargetType() == 1) {
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
//        } else if(drive.robot.getStackTracker().getPoleTargetType() == 2) {
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
//        } else if(drive.robot.getStackTracker().getPoleTargetType() == 3){
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
//        }
//        else{
//            getLights().setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
//        }
    }

    @Override
    public void stop() {
        super.stop();
        drive.getExpansionHubs().stop();
        drive.robot.getFeeder().stop();
    }

    public TimeProfiler getMatchRuntime() {
        return matchRuntime;
    }

    public void setMatchRuntime(TimeProfiler matchRuntime) {
        this.matchRuntime = matchRuntime;
    }

    public static RevBlinkinLedDriver getLights() {
        return lights;
    }

    public static void setLights(RevBlinkinLedDriver lights) {
        PPTeleopRobotFeeder.lights = lights;
    }

}
