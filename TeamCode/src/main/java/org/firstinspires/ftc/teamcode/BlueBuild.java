package org.firstinspires.ftc.teamcode;
/*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="Blue: Build", group="Build")
public class BlueBuild extends OpMode{

    private int stateMachineFlow;
    MecanumDrive robot = new MecanumDrive();
    Intake intake      = new Intake();
    Lift lift          = new Lift();
    Placing placing    = new Placing();
    SkystoneCam cam    = new SkystoneCam();

    double time;
    double distanceToTarget;
    static final int NINETY_DEGREES = 90;
    ParkPosition parkPosition = ParkPosition.MID;
    boolean endPositionUpdated;
    private ElapsedTime     runtime = new ElapsedTime();
    /***********************************
     *
     * This program starts with the robot's intake facing the wall
     *
     ***********************************/
    /*@Override
    public void init() {
        msStuckDetectInit = 11500;
        msStuckDetectLoop = 10000;

        robot.init(hardwareMap);
        robot.initIMU(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);
        placing.init(hardwareMap);
        //cam.init(hardwareMap);

        stateMachineFlow = 0;
    }
    public void init_loop() {
        if (gamepad2.dpad_up) {
            endPositionUpdated = true;
        } else if (!gamepad2.dpad_up && endPositionUpdated && parkPosition == ParkPosition.WALL) {
            parkPosition = ParkPosition.MID;
            endPositionUpdated = false;
        } else if (!gamepad2.dpad_up && endPositionUpdated && parkPosition == ParkPosition.MID) {
            parkPosition = ParkPosition.WALL;
            endPositionUpdated = false;
        }
        telemetry.addData("End Position",parkPosition);
        telemetry.update();
    }

    @Override
    public void loop() {
        switch (stateMachineFlow) {
            case 0:
                runtime.reset();
                time = getRuntime();
                stateMachineFlow++;
                break;
            case 1:
                robot.linearDrive(.5,-5);
                stateMachineFlow++;
                break;
            case 2:
                //Move right
                robot.sideDrive(.45,22);
                stateMachineFlow++;
                break;
            case 3:
                //Back up to the foundation
                robot.linearDrive(.5, -25);
                stateMachineFlow++;
                break;
            case 4:
                placing.setPlateHooks(ServoPosition.DOWN);
                stateMachineFlow++;
                break;
            case 5:
                //move the plate into the zone
                robot.linearDrive(.4,27);
                stateMachineFlow++;
                break;
            case 6:
                robot.gStatTurn(.6,90);
                stateMachineFlow++;
                break;
            case 7:
                placing.setPlateHooks(ServoPosition.UP);
                stateMachineFlow++;
                break;
            case 8:
                robot.linearDrive(.5,-8);
                stateMachineFlow++;
                break;
            case 9:
                robot.linearDrive(.45,5);
                stateMachineFlow++;
                break;
            case 10:
                if (parkPosition == ParkPosition.WALL){
                    robot.sideDrive(.5,6);
                }else if (parkPosition == ParkPosition.MID){
                    robot.sideDrive(.5,-8);
                }
                stateMachineFlow++;
                break;
            case 11:
                //move under the bridge
                robot.linearDrive(.6,40);
                stateMachineFlow++;
                break;
        }
    }
}
*/