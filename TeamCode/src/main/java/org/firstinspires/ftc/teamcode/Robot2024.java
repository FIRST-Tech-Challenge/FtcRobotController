package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.utilities.CASH_Drive_Library;
import org.firstinspires.ftc.teamcode.utilities.ElevatorControl;
import org.firstinspires.ftc.teamcode.utilities.GenericContorl_Template;
import org.firstinspires.ftc.teamcode.utilities.IMUUtility;
import org.firstinspires.ftc.teamcode.utilities.SweeperControl;

public class Robot2024<_opMode> {
    public final OpMode _opMode;
    public Robot2024(OpMode opMode) {
        _opMode = opMode;
    }
    public DcMotor leftFrontMotor = null;
    public DcMotor rightFrontMotor = null;
    public DcMotor leftRearMotor = null;
    public DcMotor rightRearMotor = null;
    //Creates a new object for the robot IMU
    public IMUUtility robotIMU = new IMUUtility();
    //Creates a new object for the Drive Library
    public CASH_Drive_Library CASHDriveLibrary = new CASH_Drive_Library();
    //Creates a new object of type Generic Control
    private GenericContorl_Template GenericControl;
    {
        GenericControl = new GenericContorl_Template();
    }
    private ElevatorControl elevatorCode;
    {
        elevatorCode = new ElevatorControl();
    }

    private SweeperControl SweeperCode;
    {
       SweeperCode = new SweeperControl();
    }
    /*
    //Control directions of the robot.  These should not be changed as these are specific to how the robot is designed
    */
    public double FORWARD = CASHDriveLibrary.FORWARD;
    public double REVERSE = CASHDriveLibrary.REVERSE;
    public double RIGHT = CASHDriveLibrary.RIGHT;
    public double LEFT = CASHDriveLibrary.LEFT;

    public double TURN_RIGHT = CASHDriveLibrary.TURN_RIGHT;
    public double TURN_LEFT = CASHDriveLibrary.TURN_LEFT;

    public int NinetyDegreeTurn = 7;
    /*
    End of direction definitions
     */

    public void initializeRobot() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightRearMotor = _opMode.hardwareMap.get(DcMotor.class, "right_rear_drive");
        rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        rightRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftRearMotor = _opMode.hardwareMap.get(DcMotor.class, "left_rear_drive");
        leftRearMotor.setDirection(DcMotor.Direction.FORWARD);
        leftRearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        leftFrontMotor = _opMode.hardwareMap.get(DcMotor.class, "left_front_drive");
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        rightFrontMotor = _opMode.hardwareMap.get(DcMotor.class, "right_front_drive");
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        //Initialize objects in common lib
        CASHDriveLibrary.init(_opMode);
        CASHDriveLibrary.leftFrontMotor = leftFrontMotor;
        CASHDriveLibrary.rightFrontMotor = rightFrontMotor;
        CASHDriveLibrary.leftRearMotor = leftRearMotor;
        CASHDriveLibrary.rightRearMotor = rightRearMotor;
        CASHDriveLibrary.EnableEncoders();

        robotIMU.initialize(_opMode,"imu");
        CASHDriveLibrary.imu = robotIMU;

        // Initilize GenericControl_Template
        // GenericControl.init(_opMode,"generic_motor");


    }
   public void initializeImplements() {
       elevatorCode.init(_opMode, "elevator_motor");
        SweeperCode.init(_opMode, "sweeper_motor");

    }
    public void initializeImplements2() {
        elevatorCode.init2(_opMode, "elevator_motor");
        SweeperCode.init(_opMode, "sweeper_motor");
    }

    ////////////////////////////////////////New Navigation Methods////////////////////////////////

    public void moveRobotteli(double leftjoyx, double leftjoyy, double rightjoyx) {
        // CASHDriveLibrary.EnableEncoders();
//        CASHDriveLibrary.resetForeAftEncoder();
        CASHDriveLibrary.MoveRobotTeliOp(leftjoyx, leftjoyy, rightjoyx,  false, false);
    }
    //Used to move the robot forward/revers/left/right.  This also uses fore/aft encoder and imu
    //for corrections.  IMU keeps the heading constant and fore/aft encoder helps keeps the
    //robot going in a straight line left and right.
    public void moveRobotAuto(double direction_deg, double power, double distance_inch) {
        CASHDriveLibrary.EnableEncoders();
//        CASHDriveLibrary.resetForeAftEncoder();
        CASHDriveLibrary.MoveRobotAuto(direction_deg, power, distance_inch, false, _opMode, false);
    }

    public void rotateRobotAuto2(double direction, double rotationAngle_d, double power) {
        CASHDriveLibrary.RotateRobotAuto2(direction, rotationAngle_d, power);
    }
    public void navWTgs(double headingError){
        CASHDriveLibrary.navigateAprilTags(headingError);
    }
    public int getTicks(){
      return  CASHDriveLibrary.getRightRearEncoderTick();

    }

    public void testDriveMotors(double test_power, long num_milliseconds, int motorNum) throws InterruptedException {
        switch( motorNum ) {
            case 0:
                CASHDriveLibrary.rightRearMotor.setPower(test_power);
                break;
            case 1:
                CASHDriveLibrary.leftRearMotor.setPower(test_power);
                break;
            case 2:
                CASHDriveLibrary.rightFrontMotor.setPower(test_power);
                break;
            case 3:
                CASHDriveLibrary.leftFrontMotor.setPower(test_power);
                break;

        }

        try {
            Thread.sleep (num_milliseconds);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        CASHDriveLibrary.Stop();
    }
    ////////////////////////////////All Functions to manipulate apparatuses on robot//////////////

    //Generic Controls
    public void controlGenericMotor(double cmd) {
        GenericControl.operateMotor(cmd);
    }
    //Auto Controls
    public void controlGenericMotorAuto(double cmd, int encoderPosition){
        GenericControl.operateMotorAuto(Math.abs(cmd), encoderPosition);
    }
    public int getGenericMotorEncoderPostion(){
        return GenericControl.getCurrentPostion();}

    public boolean isGenericBusy(){
        return GenericControl.is_motor_name_variable_busy();
    }

    //elevator control functions
    public void raiseLowerElevator(double cmd) {
        elevatorCode.raiseLowerElevator_T(cmd);
    }
    public int getElevatorPositition(){
        return elevatorCode.getCurrentPostion();
    }
    public void raiseElevatorToPosition (double cmd, int DesiredPos) {
        elevatorCode.raiseLowerElevatorToPosition_AUTO(cmd, DesiredPos);
    }

    public void setDesElevatorPosition(int desHoldPosition){
        elevatorCode.set_elevator_desired_position(desHoldPosition);
    }
    public void elevatorUpdate(double dt){
        elevatorCode.updatePosControl(dt);
    }
    public void rollSweeperOut(double cmd) {
       SweeperCode.rollSweeperOut_T(cmd);
    }
    public void dump_bucket () {
        elevatorCode.dump_pixle();
        RobotLog.d(String.format("inrobot.dumpbucket"));

    }
    public void raise_bucket () {
        elevatorCode.reset_buckt();
    }

    public void launch_drone () {
        elevatorCode.release();
            }
    public void reset_launch () {
        elevatorCode.load();
    }

}

