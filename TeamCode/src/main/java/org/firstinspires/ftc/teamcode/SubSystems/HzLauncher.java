package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

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
    public double launchMotorVelocity;

    public double launcherMotorPower;
    public static final double FLYWHEEL_SUPPLY_MODE_SPEED = 0.1;
    public static final double FLYWHEEL_NOMINAL_POWER_HIGH_GOAL = 0.70;
    public static final double FLYWHEEL_NOMINAL_POWER_POWERSHOT = 0.66;
    public static final double PLUNGER_LAUNCH_POSITION = 0.67;
    public static final double PLUNGER_REST_POSITION = 0.84;


    private boolean LauncherController;

    public enum LAUNCHER_FLYWHEEL_CONTROL {
        RUNNING_FOR_SUPPLY,
        RUNNING_FOR_TARGET,
        STOPPED
    }

    public LAUNCHER_FLYWHEEL_CONTROL launcherState = LAUNCHER_FLYWHEEL_CONTROL.STOPPED;

    public HzLauncher(HardwareMap hardwareMap) {
        //Parameter Initialization
        launcherRingPlungerServo = hardwareMap.servo.get("launch_servo");
        //launcherFlyWheelMotor = hardwareMap.dcMotor.get("launch_backenc");
        launcherFlyWheelMotor = hardwareMap.get(DcMotorEx.class, "launch_backenc");

        //launcherFlyWheelMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcherFlyWheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //launcherFlyWheelMotor.setVelocityPIDFCoefficients(1.26, 0.126, 0, 12.6);
        //launcherFlyWheelMotor.setPositionPIDFCoefficients(5.0);

        launcherFlyWheelMotor.setVelocityPIDFCoefficients(1.63835, 0.163835, 0, 16.3835);
        launcherFlyWheelMotor.setPositionPIDFCoefficients(5.0);

        //TODO : AMJAD : Test this.. May be Float is enough

        launcherFlyWheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void initLauncher(){

    }

    //run flywheel motor at speed determined by selected target and distance from target
    public void runFlyWheelToTarget(double launcherMotorPower) {
        launcherFlyWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherFlyWheelMotor.setPower(launcherMotorPower);
        launchMotorVelocity = launcherFlyWheelMotor.getVelocity();
        //TODO : AMJAD : Determine if velocity encoder needs to be run with PID control
        launcherState = LAUNCHER_FLYWHEEL_CONTROL.RUNNING_FOR_TARGET;
    }

    //stop flywheel motor
    public void stopFlyWheel() {
        launcherFlyWheelMotor.setPower(0.0);
        launcherState = LAUNCHER_FLYWHEEL_CONTROL.STOPPED;
    }

    //run flywheel motor at speed determined by selected target and distance from target
    public void runFlyWheelToSupply(double launcherMotorPower) {
        launcherFlyWheelMotor.setDirection(DcMotor.Direction.FORWARD);
        launcherFlyWheelMotor.setPower(launcherMotorPower);
        //TODO : AMJAD : Determine if velocity encoder needs to be run with PID control
        launcherState = LAUNCHER_FLYWHEEL_CONTROL.RUNNING_FOR_SUPPLY;
    }

    //run launcherRingPlungerServo to push Ring from magazine to Flywheel and retract to initial state
    public void plungeRingToFlyWheel() {
        launcherRingPlungerServo.setPosition(PLUNGER_LAUNCH_POSITION);
        try {
            sleep(250);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        //TODO : AMJAD : Test if there is sleep required in between actions
        launcherRingPlungerServo.setPosition(PLUNGER_REST_POSITION);
    }

    public LAUNCHER_FLYWHEEL_CONTROL getLauncherState(){
        return launcherState;
    }

}


