package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static java.lang.Thread.sleep;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.HashMap;
import java.util.Map;

public class BrainStemRobot {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor.RunMode currentDrivetrainMode;

    private Telemetry telemetry;
    private OpMode opMode;

    // declare robot components
    public Turret turret;
    public Lift lift;
    public Extension arm;
    public SampleMecanumDrive drive;
    public Grabber grabber;
    private Map stateMap;

    private final String CONE_CYCLE_IN_PROGRESS = "IN PROGRESS";
    private final String CONE_CYCLE_COMPLETE = "COMPLETE";
    private final String CONE_CYCLE = "CONE CYCLE";

    public BrainStemRobot(HardwareMap hwMap, Telemetry telemetry, Map stateMap) {
        this.telemetry = telemetry;
        this.stateMap = stateMap;
        this.opMode = opMode;

        // instantiate components turret, lift, arm, grabber
        turret  = new Turret(hwMap, telemetry);
        lift    = new Lift(hwMap, telemetry);
        arm     = new Extension(hwMap, telemetry);
        drive   = new SampleMecanumDrive(hwMap);
        grabber   = new Grabber(hwMap, telemetry);

        stateMap.put(CONE_CYCLE, CONE_CYCLE_COMPLETE);

        telemetry.addData("Robot", " Is Ready");
        telemetry.update();
    }

    public void initializeRobotPosition(){
        lift.initializePosition();
        lift.moveToMinHeight();  // Raise lift to clear side panels. This does not clear the arm holding cone.
        arm.getToClear();   // Extend the arm so it clears corner of the robot when swinging
        turret.initializePosition();
        lift.raiseHeightTo(0);


    }

    public void updateSystems() {
        telemetry.addData("robotStateMap" , stateMap);
        lift.setState((String) stateMap.get(lift.LIFT_SYSTEM_NAME), (String) stateMap.get(lift.LIFT_SUBHEIGHT));
        grabber.setState((String) stateMap.get(grabber.SYSTEM_NAME));
        turret.setState((String) stateMap.get(turret.SYSTEM_NAME), lift);
    }

    public void coneCycle() {
        String grabberDesiredState =  null;
        if (lift.isCollectionHeight()) {
            grabberDesiredState = grabber.CLOSED_STATE;
        } else {
            grabberDesiredState = grabber.OPEN_STATE;
        }

        stateMap.put(lift.LIFT_SUBHEIGHT, lift.PLACEMENT_HEIGHT);


        stateMap.put(lift.LIFT_SUBHEIGHT, lift.PLACEMENT_HEIGHT);
        stateMap.put(grabber.SYSTEM_NAME, grabberDesiredState);
        stateMap.put(lift.LIFT_SUBHEIGHT, lift.APPROACH_HEIGHT);
    }
}

