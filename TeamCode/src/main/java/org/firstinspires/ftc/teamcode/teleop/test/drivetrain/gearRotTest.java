package org.firstinspires.ftc.teamcode.teleop.test.drivetrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Gear Rot Test", group="Drive")
//@Disabled
public class gearRotTest extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();

    public enum DriveType{
        TOP,
        BOTTOM,
        BOTH
    }
    DriveType dtype = DriveType.BOTH;

    int targetClicks;
    double power = 0.4;

    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    /** The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
    View relativeLayout;

    @Override
    public void init() { //When "init" is clicked
        robot.init(hardwareMap);

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
        UpdatePlayer1(90);
        UpdatePlayer2();
        UpdateButton();
        UpdateTelemetry();
    }

    void UpdatePlayer1(int degrees){
        if (x.getState() == Button.State.TAP){
            dtype = DriveType.TOP;
            moveTopOne(degrees);
        } else if (y.getState() == Button.State.TAP){
            dtype = DriveType.BOTTOM;
            moveBotOne(degrees);
        } else if (a.getState() == Button.State.TAP){
            dtype = DriveType.BOTH;
            moveBoth(degrees);
        }

        DriveTrainMove();
    }

    void UpdatePlayer2(){
    }

    void UpdateTelemetry(){

        telemetry.addData("X gamepad", gamepad1.left_stick_x);
        telemetry.addData("Y gamepad", -gamepad1.left_stick_y);

        telemetry.addData("topL Clicks", robot.topL.getCurrentPosition());
        telemetry.addData("botL Clicks", robot.botL.getCurrentPosition());
        telemetry.addData("topR Clicks", robot.topR.getCurrentPosition());
        telemetry.addData("botR Clicks", robot.botR.getCurrentPosition());

        telemetry.addData("Target Clicks", targetClicks);
        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }

    public void DriveTrainMove(){
        switch(dtype){
            case TOP:
                robot.topL.setPower(power);
                robot.topR.setPower(power);
                break;

            case BOTTOM:
                robot.botL.setPower(power);
                robot.botR.setPower(power);
                break;

            case BOTH:
                robot.topL.setPower(power);
                robot.botL.setPower(power);
                robot.topR.setPower(power);
                robot.botR.setPower(power);
                break;
        }
    }

    private void moveTopOne(int degrees){
        targetClicks = (int)(degrees * constants.CLICKS_PER_DEGREE);

        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void moveBotOne(int degrees) {
        targetClicks = (int) (degrees * constants.CLICKS_PER_DEGREE);

        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks);

        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void moveBoth(int degrees){
        targetClicks = (int)(degrees * constants.CLICKS_PER_DEGREE);

        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks);
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}