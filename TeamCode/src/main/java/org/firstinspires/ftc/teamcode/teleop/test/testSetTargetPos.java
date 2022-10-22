package org.firstinspires.ftc.teamcode.teleop.test;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.common.Kinematics.TeleopKinematics;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Test setTargetPos", group="Drive")
//@Disabled
public class testSetTargetPos extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    GlobalPosSystem posSystem;
    TeleopKinematics kinematics;
    private double[] posData = new double[4];

    private ElapsedTime runtime = new ElapsedTime();

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
        kinematics = new TeleopKinematics(posSystem);
        posSystem.grabKinematics(kinematics);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        //  robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        telemetry.addData("TopL Clicks", robot.topL.getCurrentPosition());
        telemetry.addData("BotL Clicks", robot.botL.getCurrentPosition());

        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
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
        if (a.getState() == Button.State.TAP){
            setTargetSpinPosTest(240);
            robot.botL.setPower(0.5);
            robot.topL.setPower(0.5);
        } else if (a.getState() == Button.State.DOUBLE_TAP){
            setTargetRotatePosTest(340);
            robot.botL.setPower(0.5);
            robot.topL.setPower(0.5);
        } else if (b.getState() == Button.State.TAP){
            robot.botL.setPower(0);
            robot.topL.setPower(0);
        }

        //the goal of this test is to see whether or not the motor actually stops spinning after reaching its target.
        /*
        If it does, then we need to make the amount of clicks the robot rotates EXACT.  (But there's still room to consider the formula in SwerveCode for rotating).

        If it doesn't, we can just use the formula in SwerveCode (I think).
         */

        /*
        Additionally, if the motor rotates 90 degrees while testing the rotation, then we know that the Constant for degrees per click is correct.
         */
    }

    private void setTargetSpinPosTest(int clicks){
        int botLCurrPos = robot.botL.getCurrentPosition();
        int topLCurrPos = robot.topL.getCurrentPosition();
        robot.botL.setTargetPosition(botLCurrPos + clicks);
        robot.topL.setTargetPosition(topLCurrPos + clicks);

        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setTargetRotatePosTest(int clicks){
        robot.botL.setTargetPosition(-clicks);
        robot.topL.setTargetPosition(clicks);

        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void reset(){
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}