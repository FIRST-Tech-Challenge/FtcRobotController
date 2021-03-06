package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static java.lang.Thread.sleep;

/*Class Description:Launcher consists of a plunger mechanism to push the ring
(controlled by a servomotor) to a flywheel system (which is rotating with one or
two motors). Speed on the flywheel is modifiable to reach different distances and
height of target.
 */
public class HzLauncher {

    //Object declaration
    public Servo launcherRingPlungerServo;
    public DcMotorEx launcherFlyWheelMotor;

    public static final double FLYWHEEL_SUPPLY_MODE_SPEED = 0.1;
    public double FLYWHEEL_NOMINAL_VELOCITY_HIGH_GOAL = 1500;
    public double FLYWHEEL_NOMINAL_VELOCITY_POWERSHOT = 1400;
    public double FLYWHEEL_BATTERY_CORRECTION = 20;
    public double flyWheelVelocityHighGoal = FLYWHEEL_NOMINAL_VELOCITY_HIGH_GOAL;
    public double flyWheelVelocityPowerShot = FLYWHEEL_NOMINAL_VELOCITY_POWERSHOT;
    public double DELTA_VELOCITY_CORRECTION = 30;
    public static final double PLUNGER_LAUNCH_POSITION = 0.65;
    public static final double PLUNGER_REST_POSITION = 0.84;
    public static final double PLUNGER_PRELAUNCH_POSITION = 0.86;


    public enum LAUNCHER_FLYWHEEL_CONTROL {
        RUNNING_FOR_SUPPLY,
        RUNNING_FOR_TARGET,
        STOPPED
    }

    public LAUNCHER_FLYWHEEL_CONTROL launcherState = LAUNCHER_FLYWHEEL_CONTROL.STOPPED;

    public HzLauncher(HardwareMap hardwareMap) {
        //Parameter Initialization
        launcherRingPlungerServo = hardwareMap.servo.get("launch_servo");
        launcherFlyWheelMotor = hardwareMap.get(DcMotorEx.class, "launch_backenc");

        launcherFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //launcherFlyWheelMotor.setVelocityPIDFCoefficients(1.63835, 0.163835, 0, 16.3835);
        launcherFlyWheelMotor.setVelocityPIDFCoefficients(8.0, 0.163835, 0, 16.3835);
        launcherFlyWheelMotor.setPositionPIDFCoefficients(8.0);

        launcherFlyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherRingPlungerServo.setPosition(PLUNGER_REST_POSITION);
    }

    /**
     * run flywheel motor at speed determined by selected target and distance from target
     */

    public void runFlyWheelToTarget(double launcherMotorVelocity) {
        launcherFlyWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherFlyWheelMotor.setVelocity(launcherMotorVelocity);
        launcherState = LAUNCHER_FLYWHEEL_CONTROL.RUNNING_FOR_TARGET;
    }

    /**
     * stop flywheel motor
     */
    public void stopFlyWheel() {
        launcherFlyWheelMotor.setVelocity(0);
        launcherState = LAUNCHER_FLYWHEEL_CONTROL.STOPPED;
    }

    /**
     * run flywheel motor at speed determined by selected target and distance from target
     */
    public void runFlyWheelToSupply(double launcherMotorVelocity) {
        launcherFlyWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherFlyWheelMotor.setVelocity(launcherMotorVelocity);
        launcherState = LAUNCHER_FLYWHEEL_CONTROL.RUNNING_FOR_SUPPLY;
    }

    /**
     * run launcherRingPlungerServo to push Ring from magazine to Flywheel and retract to initial state
     */
    public void plungeRingToFlyWheel() {
        launcherRingPlungerServo.setPosition(PLUNGER_PRELAUNCH_POSITION);
        try {
            sleep(200);
        } catch (InterruptedException e) {
            e.printStackTrace();
        };
        launcherRingPlungerServo.setPosition(PLUNGER_LAUNCH_POSITION);
        //plungeWait(250);
        try {
            sleep(200);//250
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        launcherRingPlungerServo.setPosition(PLUNGER_REST_POSITION);
    }


    /**
     * get Launcher State
     */
    public LAUNCHER_FLYWHEEL_CONTROL getLauncherState(){
        return launcherState;
    }

    /**
     * Plunger wait
     */
    public void plungeWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (timer.time() < time){
        }
    }

}


