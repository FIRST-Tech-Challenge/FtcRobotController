package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static java.lang.Thread.sleep;

/*Class Description:Launcher consists of a plunger mechanism to push the ring
(controlled by a servomotor) to a flywheel system (which is rotating with one or
two motors). Speed on the flywheel is modifiable to reach different distances and
height of target.
 */
public class Launcher {

    //Object declaration
    public Servo launcherRingPlungerServo;
    public DcMotor launcherFlyWheelMotor;

    float arrayForHeight;
    float arrayForDistance;
    float launcherServoPosition;
    public double launcherMotorPower;
    public static final double FLYWHEEL_SUPPLY_MODE_SPEED = 0.1;
    public static final double PLUNGER_LAUNCH_POSITION = 0.5; //TODO : AMJAD : Test and fix value
    public static final double PLUNGER_REST_POSITION = 0.0; //TODO : AMJAD : Test and fix value

    private boolean LauncherController;

    public enum LAUNCHER_FLYWHEEL_CONTROL {
        FLYWHEEL_RUNNING_FOR_SUPPLY,
        FLYWHEEL_RUNNING_FOR_TARGET,
        FLYWHEEL_STOPPED
    }

    public LAUNCHER_FLYWHEEL_CONTROL launcherState = LAUNCHER_FLYWHEEL_CONTROL.FLYWHEEL_STOPPED;

    public Launcher(HardwareMap hardwareMap) {
        //Parameter Initialization
        launcherRingPlungerServo = hardwareMap.servo.get("launch_servo");
        launcherFlyWheelMotor = hardwareMap.dcMotor.get("launch_motor");
        launcherFlyWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //TODO : AMJAD : Test this.. May be Float is enough
        launcherFlyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void initLauncher(){

    }

    //run flywheel motor at speed determined by selected target and distance from target
    public void runFlyWheelToTarget(double launcherMotorPower) {
        launcherFlyWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherFlyWheelMotor.setPower(launcherMotorPower);
        //TODO : AMJAD : Determine if velocity encoder needs to be run with PID control
        launcherState = LAUNCHER_FLYWHEEL_CONTROL.FLYWHEEL_RUNNING_FOR_TARGET;
    }

    //stop flywheel motor
    public void stopFlyWheel() {
        launcherFlyWheelMotor.setPower(0.0);
        launcherState = LAUNCHER_FLYWHEEL_CONTROL.FLYWHEEL_STOPPED;
    }

    //run flywheel motor at speed determined by selected target and distance from target
    public void runFlyWheelToSupply(double launcherMotorPower) {
        launcherFlyWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherFlyWheelMotor.setPower(launcherMotorPower);
        //TODO : AMJAD : Determine if velocity encoder needs to be run with PID control
        launcherState = LAUNCHER_FLYWHEEL_CONTROL.FLYWHEEL_RUNNING_FOR_SUPPLY;
    }

    //run launcherRingPlungerServo to push Ring from magazine to Flywheel and retract to initial state
    public void plungeRingToFlyWheel() {
        launcherRingPlungerServo.setPosition(PLUNGER_LAUNCH_POSITION);
        try {
            sleep(500);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //TODO : AMJAD : Test if there is sleep required in between actions
        launcherRingPlungerServo.setPosition(PLUNGER_REST_POSITION);
    }

    public LAUNCHER_FLYWHEEL_CONTROL getLauncherState(){
        return launcherState;
    }

    /*public void flywheelStatus(boolean dpadUpPressed, Magazine.MAGAZINE_POSITION magazinePos) {
        if(magazinePos == Magazine.MAGAZINE_POSITION.MAGAZINE_AT_LAUNCH) {
            if(dpadUpPressed) {
                launcherFlyWheelMotor.setPower(FLYWHEEL_SUPPLY_MODE_SPEED);
                //I don't know how to set the state to FLYWHEEL_RUNNING_FOR_SUPPLY.
            }

        }

    }*/

}


