package org.firstinspires.ftc.teamcode;
/*
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@Autonomous(name="Blue: Stones", group="Test")
public class BlueStone extends OpMode {

    private int stateMachineFlow;
    //the below int controls whether we move the plate (0 = move, 1 = is moved by teammates)
    static final int PLATE_IS_MOVED = 1;
    MecanumDrive robot = new MecanumDrive();
    Intake intake      = new Intake();
    Lift lift          = new Lift();
    Placing placing    = new Placing();
    SkystoneCam cam    = new SkystoneCam();

    double time;
    double distanceToTarget;
    static final double CAMERA_LENS_POS = 0;
    static final double NO_INPUT_DISTANCE = 10;
    static final int NINETY_DEGREES = 90;
    static final double DISTANCE_TO_WALL = 10;
    int angleToTarget = 0;
    int initView = 0;
    private ElapsedTime runtime = new ElapsedTime();
    /***********************************
     *
     * This program starts with the robot's left side(left of the intake) facing the field
     *
     ***********************************/
    /*@Override
    public void init() {
        msStuckDetectInit = 13500;
        msStuckDetectLoop = 11500;

        robot.init(hardwareMap);
        robot.initIMU(hardwareMap);
        intake.init(hardwareMap);
        lift.init(hardwareMap);
        placing.init(hardwareMap);
        cam.init(hardwareMap);

        stateMachineFlow = 0;
    }

    @Override
    public void loop() {
        switch (stateMachineFlow){
            case 0:
                runtime.reset();
                time = getRuntime();
                stateMachineFlow++;
                break;
            case 1:
                //Move towards stones
                robot.sideDrive(.65,11);
                stateMachineFlow++;
                break;
            case 2:
                if (cam.isVisible()) {
                    angleToTarget = Math.round(cam.getHeading()-NINETY_DEGREES);
                    robot.gStatTurn(.65, angleToTarget);
                    distanceToTarget = cam.getXPosition()-CAMERA_LENS_POS;
                }else {
                    angleToTarget = -90;
                    robot.gStatTurn(.65, angleToTarget);
                    distanceToTarget = NO_INPUT_DISTANCE;
                }
                stateMachineFlow++;
                break;
            case 3:
                robot.linearDrive(.65, -distanceToTarget);
                stateMachineFlow++;
                break;
            case 4:
                //square back to the stones
                robot.gStatTurn(.65, -NINETY_DEGREES-angleToTarget);
                stateMachineFlow++;
                break;
            case 5:
                //Move grabber into position
                placing.setClawWrist(ServoPosition.UP);
                //placing.setClawTurn(ServoPosition.TURN_OUT);
                placing.setClawWrist(ServoPosition.DOWN);
                stateMachineFlow++;
                break;
            case 6:
                //Back up
                robot.linearDrive(.65,-3);

                stateMachineFlow++;
                break;
            case 7:
                //grab stone and raise it off the floor
                placing.setClawGrip(ServoPosition.DOWN);
                lift.placeLevel(PlaceLevel.TWO);
            case 8:
                //Move away from the other stones
                robot.linearDrive(.5,3);
                stateMachineFlow++;
                break;
            case 9:
                robot.gStatTurn(.65, -(-NINETY_DEGREES-angleToTarget));
                stateMachineFlow++;
                break;
            case 10:
                robot.linearDrive(.65, distanceToTarget);
                stateMachineFlow++;
                break;
            case 11:
                robot.gStatTurn(.65,-angleToTarget);
                stateMachineFlow++;
                break;
            case 12:
                //Move to other side of bridge
                robot.linearDrive(-.65,-54);
                stateMachineFlow++;
                break;
            case 13:
                //move to foundation
                if (PLATE_IS_MOVED == 0){
                    robot.sideDrive(.65,-34.5);
                }else if (PLATE_IS_MOVED == 1){
                    robot.sideDrive(.65,-8);
                }
                stateMachineFlow++;
                break;
            case 14:
                //move up to the side of the foundation
                robot.linearDrive(.65,-2.5);
                stateMachineFlow++;
                break;
            case 15:
                //drop the stone
                placing.setClawGrip(ServoPosition.UP);
                stateMachineFlow++;
                break;
            case 16:
                if (PLATE_IS_MOVED == 0){
                    //move to the side of the foundation
                    robot.linearDrive(.65,2.5);
                    robot.sideDrive(.65,18);
                }else if (PLATE_IS_MOVED == 1){
                    //move to the side to be in position to park under the bridge
                    robot.sideDrive(.65,13);
                }
                stateMachineFlow++;
                break;
            case 17:
                if (PLATE_IS_MOVED ==0){
                    //turn so back is parallel to the plate
                    robot.gStatTurn(.65,-NINETY_DEGREES);
                    stateMachineFlow++;
                }else if (PLATE_IS_MOVED == 1){
                    //drive under the bridge and finish
                    robot.linearDrive(.65,45);
                    stateMachineFlow = 30;
                }
                break;
            case 18:
                //move to the side of the plate and up to it
                robot.sideDrive(.65,30);
                robot.linearDrive(.65,-4.5);
                stateMachineFlow++;
                break;
            case 19:
                //set hooks and pull plate
                placing.setPlateHooks(ServoPosition.DOWN);
                stateMachineFlow++;
                break;
            case 20:
                robot.linearDrive(.65,29);
                stateMachineFlow++;
                break;
            case 21:
                placing.setPlateHooks(ServoPosition.UP);
                stateMachineFlow++;
                break;
            case 22:
                //move under bridge
                robot.sideDrive(.65,-50);
                stateMachineFlow++;
                break;
        }
    }
}
*/