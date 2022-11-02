package org.firstinspires.ftc.teamcode.teleop.test.drivetrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Reset;
import org.firstinspires.ftc.teamcode.common.kinematics.LinearKinematicsTest;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="Test Controller", group="Drive")
//@Disabled
public class TestControllerOpMode extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
    GlobalPosSystem posSystem;
    LinearKinematicsTest kinematics;

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
        telemetry.addData("X gamepad", gamepad1.left_stick_x);
        telemetry.addData("Y gamepad", -gamepad1.left_stick_y);
        telemetry.addData("Right TargetW", kinematics.getRTargetW());
        telemetry.addData("Left TargetW", kinematics.getLTargetW());

        telemetry.addData("Right Turn Amount", kinematics.getRTurnAmount());
        telemetry.addData("Left Turn Amount", kinematics.getLTurnAmount());

        telemetry.addData("Right Optimized Target", kinematics.getROptimizedTargetW());
        telemetry.addData("Left Optimized Target", kinematics.getLOptimizedTargetW());

        telemetry.addData("Right Direction", kinematics.getRightDirectionW());
        telemetry.addData("Left Direction", kinematics.getLeftDirectionW());

        telemetry.addData("Drive Type", kinematics.getdDriveType());

        telemetry.addData("Should Snap?", kinematics.shouldSnap());

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
        double left_stick_y = -gamepad1.left_stick_y; //returns a value between [-1, 1]
        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
        double right_stick_y = -gamepad1.right_stick_y; //returns a value between [-1, 1]

        kinematics.getGamepad(left_stick_x, left_stick_y, right_stick_x, right_stick_y);

        kinematics.setPos();

        kinematics.logic();

    }


    private void setPower(){

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}