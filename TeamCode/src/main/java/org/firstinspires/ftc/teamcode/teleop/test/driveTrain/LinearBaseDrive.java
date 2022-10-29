package org.firstinspires.ftc.teamcode.teleop.test.driveTrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Kinematics.LinearKinematicsTest;
import org.firstinspires.ftc.teamcode.common.Kinematics.TeleopKinematics;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Linear Base Drive Test", group="Drive")
//@Disabled
public class LinearBaseDrive extends OpMode{
    public enum ControllerType{
        CONTOLLER,
        BUTTON,
        NOT_INITIALIZED
    }
    ControllerType controllerType = ControllerType.NOT_INITIALIZED;

    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    LinearKinematicsTest kinematics;

    private ElapsedTime runtime = new ElapsedTime();
    private double[] posData = new double[4];

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    //for resetting the robot's wheels' orientation
    ElapsedTime resetTimer = new ElapsedTime();
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        posSystem = new GlobalPosSystem(robot);
        kinematics = new LinearKinematicsTest(posSystem);
        posSystem.grabKinematics(kinematics);
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() { //When "start" is pressed

    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer1();
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    void UpdatePlayer1(){
        DriveTrainBase();
    }

    void UpdatePlayer2(){
    }

    void UpdateTelemetry(){
        for(int i = 0; i < 4; i++){
            posData[i] = posSystem.getPositionArr()[i];
        }
        telemetry.addData("X", posData[0]);
        telemetry.addData("Y", posData[1]);
        telemetry.addData("W", posData[2]);
        telemetry.addData("R", posData[3]);
        telemetry.addData("Drive Type", kinematics.getdDriveType());
        telemetry.addData("Power Top", kinematics.getPower()[0]);
        telemetry.addData("Power Bottom", kinematics.getPower()[1]);
        telemetry.addData("TargetW", kinematics.getTargetW());
        telemetry.addData("Direction", kinematics.getDirectionW());
        telemetry.addData("Turn Amount", kinematics.getTurnAmount());
        telemetry.addData("Optimized Target", kinematics.getOptimizedTargetW());
        telemetry.addData("Rot Clicks", kinematics.rotClicks);
        telemetry.addData("Spin clicks", kinematics.spinClicks);
        telemetry.addData("In cycle?", kinematics.inCycle);
        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }

    void DriveTrainBase(){
        DriveTrainMove();
    }

    private void DriveTrainMove(){
        //gps system
        posSystem.calculatePos();
        kinematics.setCurrents();

        //setting targets
        setVariables();

        //put power into the motors
        setPower();

    }

    private void setVariables(){
        //outputs of joysticks
        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
        double left_stick_y = gamepad1.left_stick_y; //returns a value between [-1, 1]
        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        double right_stick_y = gamepad1.right_stick_y; //returns a value between [-1, 1]

        if (x.getState() == Button.State.TAP) controllerType = ControllerType.CONTOLLER;
        else if (y.getState() == Button.State.TAP) controllerType = ControllerType.BUTTON;

        switch(controllerType){
            case CONTOLLER:
                kinematics.getGamepad(left_stick_x, left_stick_y, right_stick_x, right_stick_y);

                kinematics.setPos();

                kinematics.logic();
                break;

            case BUTTON:
                kinematics.getGamepad(0.5, 0.5, 0, 0);

                kinematics.setPos();

                kinematics.logic();
                break;
        }



//        reset(); //snaps wheels back to 0 degrees if the robot has stopped moving
    }



    private void setPower(){
        double[] motorPower = kinematics.getPower();

        if (motorPower[0] == 0 && motorPower[1] == 0 && motorPower[2] == 0 && motorPower[3] == 0){
            robot.setMotorPower(0);
            return;
        }

        int[] targetClicks = kinematics.getClicks();
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks[0]);
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks[1]);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks[2]);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks[3]);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.topL.setPower(motorPower[0] * constants.POWER_LIMITER);
        robot.botL.setPower(motorPower[1] * constants.POWER_LIMITER);
        robot.topR.setPower(motorPower[2] * constants.POWER_LIMITER);
        robot.botR.setPower(motorPower[3] * constants.POWER_LIMITER);

        if (motorPower[0] == 0) robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (motorPower[1] == 0) robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (motorPower[2] == 0) robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (motorPower[3] == 0) robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void reset(){

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}