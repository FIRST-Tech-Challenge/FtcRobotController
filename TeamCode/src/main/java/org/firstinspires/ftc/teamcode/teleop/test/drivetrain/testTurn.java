package org.firstinspires.ftc.teamcode.teleop.test.drivetrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Accelerator;
import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.kinematics.LinearKinematicsTest;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.kinematics.SimplifiedKinematics;
import org.firstinspires.ftc.teamcode.common.kinematics.TurnKinematicsTest;

@TeleOp(name="Test Turn", group="Drive")
//@Disabled
public class testTurn extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    GlobalPosSystem posSystem;

    Reset reset;
    Accelerator toplAccelerator = new Accelerator();
    Accelerator botlAccelerator = new Accelerator();
    Accelerator toprAccelerator = new Accelerator();
    Accelerator botrAccelerator = new Accelerator();

    //robot's power
    double leftRotatePower = 0.0;
    double rightRotatePower = 0.0;
    double spinPower = 0.0;

    //target clicks
    public int rightRotClicks = 0;
    public int leftRotClicks = 0;
    public int spinClicks = 0; //make protected later

    public int spinDirectionR = 1;
    public int spinDirectionL = 1;

    private enum Module{
        RIGHT,
        LEFT
    }

    //current orientation
    double leftCurrentW; //current wheel orientation
    double rightCurrentW;
    double currentR; //current robot header orientation

    double rTrigger = 0;
    double lTrigger = 0;

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
        reset = new Reset(robot);
        posSystem = new GlobalPosSystem(robot);

        telemetry.addData("Say", "Hello Driver");
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

    }

    void UpdatePlayer2(){
    }

    void UpdateTelemetry(){
        double[] posData = posSystem.getPositionArr();

        telemetry.addData("X gamepad", gamepad1.left_stick_x);
        telemetry.addData("Y gamepad", -gamepad1.left_stick_y);
        telemetry.addData("Left trigger", gamepad1.left_trigger);
        telemetry.addData("Right trigger", -gamepad1.right_trigger);
        telemetry.addData("X", posData[0]);
        telemetry.addData("Y", posData[1]);
        telemetry.addData("Left W", posData[2]);
        telemetry.addData("Right W", posData[3]);
        telemetry.addData("R", posData[4]);
        telemetry.addData("Right Rot Target Clicks", rightRotClicks);
        telemetry.addData("Left Rot Target Clicks", leftRotClicks);
        telemetry.addData("Target Spin clicks", spinClicks);
        telemetry.addData("Right Spin Direction", spinDirectionR);
        telemetry.addData("Left Spin Direction", spinDirectionL);
        telemetry.addData("topL Clicks", robot.topL.getCurrentPosition());
        telemetry.addData("botL Clicks", robot.botL.getCurrentPosition());

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

        ///outputs of joysticks
        rTrigger = gamepad1.right_trigger;
        lTrigger = gamepad1.left_trigger;

        int direction = (rTrigger > lTrigger ? 1 : -1);
        spinDirectionL = (direction == 1 ? 1 : -1);
        spinDirectionR = (direction == 1 ? -1 : 1);

        double trigger = Math.max(rTrigger, lTrigger);
        logic(trigger);
    }

    public void logic(double trigger){
        leftCurrentW = posSystem.getLeftWheelW();
        rightCurrentW = posSystem.getRightWheelW();
        currentR = posSystem.getPositionArr()[4];

        if (noMovementRequests()){
            spinPower = 0;
            leftRotatePower = 0;
            rightRotatePower = 0;

            spinClicks = 0;
            rightRotClicks = 0;
            leftRotClicks = 0;

//            translatePerc = 0;
//            rotatePerc = 0;
        } else if(shouldTurn()){
            spinPower = trigger;
            leftRotatePower = 0;
            rightRotatePower = 0;

            spinClicks = (int)(spinPower * 100); //currently, the robot won't stop turning once its hit its target.  WIll add that once we confirm this works.
            rightRotClicks = 0;
            leftRotClicks = 0;
        }

        int targetTopL = spinClicks * spinDirectionL; //left
        int targetBotL = -spinClicks * spinDirectionL; //left
        int targetTopR = spinClicks * spinDirectionR; //right
        int targetBotR = -spinClicks * spinDirectionR; //right
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetTopL);
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetBotL);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetTopR);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetBotR);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double powerTopL = spinPower + leftRotatePower; //top left
        double powerBotL = spinPower + leftRotatePower; //bottom left
        double powerTopR = spinPower + rightRotatePower; //top right
        double powerBotR = spinPower + rightRotatePower; //bottom right

        toplAccelerator.update(powerTopL);
        botlAccelerator.update(powerBotL);
        toprAccelerator.update(powerTopR);
        botrAccelerator.update(powerBotR);

        robot.topL.setPower(powerTopL * constants.POWER_LIMITER);
        robot.botL.setPower(powerBotL * constants.POWER_LIMITER);
        robot.topR.setPower(powerTopR * constants.POWER_LIMITER);
        robot.botR.setPower(powerBotR * constants.POWER_LIMITER);
    }

    public boolean shouldTurn(){
        return ((gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0) && (gamepad1.right_trigger != 0 || gamepad1.left_trigger != 0));
    }

    public boolean noMovementRequests(){
        return (gamepad1.left_stick_x==0 && gamepad1.left_stick_y==0 && gamepad1.right_stick_x==0 && gamepad1.right_stick_y==0 && gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0);
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < -179 || degrees > 180) {
            int modulo = (int)Math.signum(degrees) * -180;
            degrees = Math.floorMod((int)degrees, modulo);
        }
        return degrees;
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}