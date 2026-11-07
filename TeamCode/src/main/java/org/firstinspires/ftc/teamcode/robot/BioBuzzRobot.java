package org.firstinspires.ftc.teamcode.robot;

import com.broombots.sweep.Classes.Coordinate;
import com.broombots.sweep.Classes.Pos2D;
import com.broombots.sweep.Classes.SWEEPRobot;
import com.broombots.sweep.Classes.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robot.actions.ExampleAction;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Flywheel;

public class BioBuzzRobot extends SWEEPRobot {
    // Robot subsystems
    Drivetrain drivetrain;
    Flywheel flywheelPollen;

    Flywheel flywheelNectar;


    /**
     * Constructor for the SoftwareTestingBot class.
     * Initializes the robot's subsystems and takes FIRSTSDK bits
     * @param hardwareMap The hardware map used to access the robot's hardware components.
     * @param telemetry   The telemetry object used for sending data to the driver station.
     */
    //Will update numbers when testing for flywheel and nectar speeds that we need. Right now this works for Siphysius
    public BioBuzzRobot(HardwareMap hardwareMap, Telemetry telemetry){
        runtime = new ElapsedTime();
        this.drivetrain = new Drivetrain(hardwareMap, telemetry, false);
        this.flywheelPollen = new Flywheel(hardwareMap, telemetry, new ElapsedTime(),true,"pollenFlywheel");
        this.flywheelNectar = new Flywheel(hardwareMap,telemetry,new ElapsedTime(),false,"nectarFlywheel");
        subsystems = new Subsystem[]{
                drivetrain,
                flywheelPollen,
                flywheelNectar
        };
    }
    /**
     * Initializes the robot's subsystems and sets up any necessary configurations.
     * This method is called once when the robot is initialized.
     */
    public void initialize() {
        drivetrain.enableFieldCentricDriving(false);
    }
    /**
     * Updates the robot's subsystems and actions during teleop mode.
     * This method is called repeatedly during the teleop period.
     */
    public void teleopUpdate() {
        updateActions();
    }
    /**
     * Updates the robot's subsystems and actions during autonomous mode.
     * This method is called repeatedly during the autonomous period.
     */
    public void autonomousUpdate() {
        updateActions();
    }
    // Subsystem getters
    public Drivetrain getDrivetrain() {
        return drivetrain;
    }
    public Flywheel getFlywheelPollen(){
        return flywheelPollen;
    }
    // Robot action definitions
    /**
     * Creates an ExampleAction to demonstrate how actions should be defined
     * @param position The coordinate around which the action is triggered.
     * @param time       The duration for which the action runs (in seconds).
     * @param range      The trigger radius for the action (in inches).
     * @return An instance of ExampleAction.
     */
    public ExampleAction exampleAction(Pos2D position, double time, double range){
        return new ExampleAction(this, position, time, range, 0);
    }
    /*
    public SetFlywheelSpeedAction flywheelAction(Pos2D position, double time,double speed){
        return new SetFlywheelSpeedAction(this, position,time,speed);
    }
     */


    /**
     * Simplified example action which used default parameters
     * @return new exampleAction
     */
    public ExampleAction exampleAction(){
        return new ExampleAction(this);
    }
}
