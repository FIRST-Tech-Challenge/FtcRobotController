package org.firstinspires.ftc.teamcode.teleop.test.driveTrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Swerve Code", group="Drive")
//@Disabled
public class SwerveCode extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    Kinematics kinematics;
    private double[] posData = new double[4];

    private ElapsedTime runtime = new ElapsedTime();

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    private int rotateR;
    private int rotateL;

    public enum State{
        DRIVE,
        RESET
    }
    State driveState = State.DRIVE;
    boolean isResetCycle = false;

    //for resetting the robot's wheels' orientation
    ElapsedTime resetTimer = new ElapsedTime();
    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);
        posSystem = new GlobalPosSystem(robot);
        kinematics = new Kinematics(posSystem);
        posSystem.grabKinematics(kinematics);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
       // DriveTrainBasePower();
        if (driveState == State.DRIVE){
            DriveTrainPowerEncoder();
//            isResetCycle = false;
        } else if (driveState == State.RESET){
            reset();
        }

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

        telemetry.addData("topL clicks", robot.topL.getCurrentPosition());
        telemetry.addData("botL clicks", robot.botL.getCurrentPosition());
        telemetry.addData("topR clicks", robot.topR.getCurrentPosition());
        telemetry.addData("botR clicks", robot.botR.getCurrentPosition());

        telemetry.addData("rotateR target", rotateR);
        telemetry.addData("rotateL target", rotateL);
        telemetry.addData("isBusy", robot.wheelsAreBusy());
        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);

        if (x.getState() == Button.State.TAP){
            driveState = State.RESET;
        } else if (y.getState() == Button.State.TAP){
            driveState = State.DRIVE;
        }
    }

    void DriveTrainBasePower(){
        int powerBotL = 1;
        int powerTopL = 1;

        if (gamepad1.dpad_up){
            robot.botL.setPower(powerBotL);
            robot.topL.setPower(powerTopL);
        }
        else{
            robot.botL.setPower(0);
            robot.topL.setPower(0);
        }
    }

    void DriveTrainPowerEncoder(){
        posSystem.calculatePos();

        int posBotL = robot.botL.getCurrentPosition();
        int posTopL = robot.topL.getCurrentPosition();
        int posBotR = robot.botR.getCurrentPosition();
        int posTopR = robot.topR.getCurrentPosition();

        double alpha = 0.5;
        double beta = 1 - alpha;

        int distanceTopL = (int) (-gamepad1.left_stick_y * 100 * beta);
        int distanceBotL = (int) (gamepad1.left_stick_y * 100 * beta);
        int distanceTopR = distanceTopL;
        int distanceBotR = distanceBotL;

        int rotationalTopL = (int) (gamepad1.left_stick_x * 100 * alpha);
        int rotationalBotL = (int) (gamepad1.left_stick_x * 100 * alpha);
        int rotationalTopR = rotationalTopL;
        int rotationalBotR = rotationalBotL;

        robot.botL.setTargetPosition(posBotL + distanceBotL + rotationalBotL);
        robot.topL.setTargetPosition(posTopL + distanceTopL + rotationalTopL);
        robot.botR.setTargetPosition(posBotR + distanceBotR + rotationalBotR);
        robot.topR.setTargetPosition(posTopR + distanceTopR + rotationalTopR);

        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.botL.setPower(-gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha);
        robot.topL.setPower(-gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha);
        robot.botR.setPower(-gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha);
        robot.topR.setPower(-gamepad1.left_stick_y * beta + gamepad1.left_stick_x * alpha);
    }

    private void reset(){
        if (!isResetCycle){
            isResetCycle = true;
            int topR = robot.topR.getCurrentPosition();
            int botR = robot.botR.getCurrentPosition();
            int topL = robot.topL.getCurrentPosition();
            int botL = robot.botL.getCurrentPosition();

            rotateR = (topR + botR) / 2;
            rotateL = (topL + botL) / 2;

            robot.botL.setTargetPosition(robot.botL.getCurrentPosition() -rotateL);
            robot.topL.setTargetPosition(robot.topL.getCurrentPosition() -rotateL);
            robot.botR.setTargetPosition(robot.botR.getCurrentPosition() -rotateR);
            robot.topR.setTargetPosition(robot.topR.getCurrentPosition() -rotateR);

            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        else{
            robot.botL.setPower(0.2);
            robot.topL.setPower(0.2);
            robot.botR.setPower(0.2);
            robot.topR.setPower(0.2);
        }

        //make sure to reset the encoder position afterwards without messing stuff up like before.
    }



    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}