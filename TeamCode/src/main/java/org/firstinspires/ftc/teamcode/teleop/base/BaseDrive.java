package org.firstinspires.ftc.teamcode.teleop.base;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Kinematics.TeleopKinematics;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Base Drive", group="Drive")
//@Disabled
public class BaseDrive extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    TeleopKinematics kinematics;

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
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);

        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        posSystem = new GlobalPosSystem(robot);
        kinematics = new TeleopKinematics(posSystem);
        posSystem.grabKinematics(kinematics);
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"

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
        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);
        //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        for(int i = 0; i < 4; i++){
            posData[i] = posSystem.getPositionArr()[i];
        }
        telemetry.addData("Xpos", posData[0]);
        telemetry.addData("Ypos", posData[1]);
        telemetry.addData("W", posData[2]);
        telemetry.addData("R", posData[3]);
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

        kinematics.getGamepad(left_stick_x, left_stick_y, right_stick_x, right_stick_y);

        kinematics.setPos();

        kinematics.logic();

        reset(); //snaps wheels back to 0 degrees if the robot has stopped moving
    }



    private void setPower(){
        double[] motorPower = new double[4];

        motorPower[0] = kinematics.getPower()[0];
        motorPower[1] = kinematics.getPower()[1];
        motorPower[2] = kinematics.getPower()[2];
        motorPower[3] = kinematics.getPower()[3];

        if (motorPower[0] == 0 && motorPower[1] == 0 && motorPower[2] == 0 && motorPower[3] == 0){
            robot.setMotorPower(0);
            return;
        }

        robot.topL.setPower(motorPower[0]);
        robot.botL.setPower(motorPower[1]);
        robot.topR.setPower(motorPower[2]);
        robot.botR.setPower(motorPower[3]);
    }



    //!!!!!!!!MOVE THIS STUFF TO TELEOPKINEMATICS AFTER TESTING THAT GPS POS SYSTEM WORKS!!!!!!!!
    public void setRotateTargetClicks(){
        int posBotL = robot.botL.getCurrentPosition();
        int posTopL = robot.topL.getCurrentPosition();
        int posTopR = robot.topR.getCurrentPosition();
        int posBotR = robot.botR.getCurrentPosition();

        double alpha = 0.5;
        double beta = 1 - alpha;

        int distanceTopL = (int) (gamepad1.left_stick_y * 100 * beta);
        int distanceBotL = (int) (-gamepad1.left_stick_y * 100 * beta);
        int distanceTopR =  distanceTopL;
        int distanceBotR = distanceBotL;

        robot.topL.setTargetPosition(posTopL + distanceTopL);
        robot.botL.setTargetPosition(posBotL + distanceBotL);
//        robot.topR.setTargetPosition(posTopR + distanceTopR);
//        robot.botR.setTargetPosition(posBotR + distanceBotR);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setSpinTargetClicks(){
        int posBotL = robot.botL.getCurrentPosition();
        int posTopL = robot.topL.getCurrentPosition();
        int posTopR = robot.topR.getCurrentPosition();
        int posBotR = robot.botR.getCurrentPosition();

        double alpha = 0.5;
        double beta = 1 - alpha;

        int rotationalTopL = (int) (gamepad1.left_stick_x * 100 * alpha);
        int rotationalBotL = (int) (gamepad1.left_stick_x * 100 * alpha);
        int rotationalTopR = rotationalTopL;
        int rotationalBotR = rotationalBotL;

        robot.topL.setTargetPosition(posTopL + rotationalTopL);
        robot.botL.setTargetPosition(posBotL + rotationalBotL);
//        robot.topR.setTargetPosition(posTopR + rotationalTopR);
//        robot.botR.setTargetPosition(posBotR + rotationalBotR);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void setTargetClicks(){
        int posBotL = robot.botL.getCurrentPosition();
        int posTopL = robot.topL.getCurrentPosition();

        double alpha = 0.7;
        double beta = 1 - alpha;

        int distanceTopL = (int) (gamepad1.left_stick_y * 100 * beta);
        int distanceBotL = (int) (-gamepad1.left_stick_y * 100 * beta);

        int rotationalTopL = (int) (gamepad1.left_stick_x * 100 * alpha);
        int rotationalBotL = (int) (gamepad1.left_stick_x * 100 * alpha);

        robot.botL.setTargetPosition(posBotL + distanceBotL + rotationalBotL);
        robot.topL.setTargetPosition(posTopL + distanceTopL + rotationalTopL);

        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.botL.setPower(gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha);
        robot.topL.setPower(gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha);
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