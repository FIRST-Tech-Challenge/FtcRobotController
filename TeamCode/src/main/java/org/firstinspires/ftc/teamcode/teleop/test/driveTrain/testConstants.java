package org.firstinspires.ftc.teamcode.teleop.test.driveTrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.common.Kinematics.TeleopKinematics;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Swerve Code", group="Drive")
//@Disabled
public class testConstants extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    Kinematics kinematics;
    private double[] posData = new double[4];

    private ElapsedTime runtime = new ElapsedTime();

    public enum State{
        SAME_DIRECTION,
        SINGLE,
        NOT_INITIALIZED
    }
    public State state = State.NOT_INITIALIZED;

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
        DriveTrainPowerEncoder();
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

    void DriveTrainPowerEncoder(){
        posSystem.calculatePos();

        double beta = 0.5;
        double alpha = 1 - beta;

        if (x.getState() == Button.State.TAP) state = State.SAME_DIRECTION;
        else if (b.getState() == Button.State.TAP) state = State.SINGLE;

        int distanceTopL = (int) (gamepad1.left_stick_y * 100 * beta);
        int distanceBotL = (int) (-gamepad1.left_stick_y * 100 * beta);

        int rotationalTopL = (int) (gamepad1.left_stick_x * 100 * alpha);
        int rotationalBotL = (int) (gamepad1.left_stick_x * 100 * alpha);

        switch (state){
            case SAME_DIRECTION:
                robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + distanceTopL + rotationalTopL);
                robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + distanceBotL + rotationalBotL);

                robot.topL.setPower(gamepad1.left_stick_y * 0.2 + gamepad1.left_stick_x * 0.2);
                robot.botL.setPower(gamepad1.left_stick_y * 0.2 + gamepad1.left_stick_x * 0.2);
                break;

            case SINGLE:
                robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + distanceTopL + rotationalTopL);
                robot.topL.setPower(gamepad1.left_stick_y * 0.2 + gamepad1.left_stick_x * 0.2);
                break;
        }



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