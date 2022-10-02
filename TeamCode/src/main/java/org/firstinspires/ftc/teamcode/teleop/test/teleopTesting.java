package org.firstinspires.ftc.teamcode.teleop.test;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Kinematics.TeleopKinematics;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;

@TeleOp(name="Test Teleop", group="Drive")

public class teleopTesting extends OpMode {
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();
//    GlobalPosSystem posSystem;
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

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

//        posSystem = new GlobalPosSystem(robot);
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void start() { //When "start" is pressed
//        telemetry.addData("X", posSystem.getPositionArr()[0]);
//        telemetry.addData("Y", posSystem.getPositionArr()[1]);
//        telemetry.addData("W", posSystem.getPositionArr()[2]);
//        telemetry.addData("R", posSystem.getPositionArr()[3]);
//        telemetry.update();
    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer1();
//        UpdatePlayer2();
//        UpdateButton();
//        UpdateTelemetry();
    }

    void UpdatePlayer1(){
        DriveTrainBase();
    }

//    void UpdatePlayer2(){
//    }

    void UpdateTelemetry(){
        telemetry.addData("X", gamepad1.left_stick_x);
        telemetry.addData("Y", -gamepad1.left_stick_y);
        telemetry.addData("R", gamepad1.right_stick_x);
        //  telemetry.addData("Touch Sensor", robot.digitalTouch.getState());
        telemetry.update();
    }
//
//    void UpdateButton(){
//        x.update(gamepad1.x);
//        y.update(gamepad1.y);
//        a.update(gamepad1.a);
//        b.update(gamepad1.b);
//    }

    void DriveTrainBase(){
        DriveTrainMove();
    }

    private void DriveTrainMove(){
//        posSystem.calculatePos();
        testMove(1, 0);
        telemetry.update();
    }


    void testMove(double r, double s){
        double rotatePower = r;
        double spinPower = s;
        double translationPowerPercentage = 0.5;
        double rotationPowerPercentage = 0.5;
        double leftThrottle = 1.0;
        double rightThrottle = 1.0;
        int rotationSwitchMotors = 1; //1 if rotating wheels right, -1 if rotating wheels left
        int translateSwitchMotors = 1; //1 if going forward, -1 if going backward

        double[] motorPower = new double[4];
        motorPower[0] = spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //top left
        motorPower[1] = -1 * spinPower * translationPowerPercentage * translateSwitchMotors * leftThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //bottom left
        motorPower[2] = spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //top right
        motorPower[3] = -1 * spinPower * translationPowerPercentage * translateSwitchMotors * rightThrottle + rotatePower * rotationPowerPercentage * rotationSwitchMotors; //bottom right

        for (int i = 0; i < 4; i++){
            robot.dtMotors[i].setPower(motorPower[i]);
        }
    }

    @Override
    public void stop() {
//        telemetry.addData("X", posSystem.getPositionArr()[0]);
//        telemetry.addData("Y", posSystem.getPositionArr()[1]);
//        telemetry.addData("W", posSystem.getPositionArr()[2]);
//        telemetry.addData("R", posSystem.getPositionArr()[3]);
//        telemetry.update();
    }
}
