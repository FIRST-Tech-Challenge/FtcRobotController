package org.firstinspires.ftc.teamcode.teleop.test.driveTrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.gps.LinearGPS;

@TeleOp(name="Test rotate GPS", group="Drive")
//@Disabled
public class testRotateGPS extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    LinearGPS posSystem;
    Constants constants = new Constants();
    private double[] posData = new double[4];

    private ElapsedTime runtime = new ElapsedTime();
    int distanceClicks;
    int rotClicks;

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
        posSystem = new LinearGPS(robot, Kinematics.DriveType.LINEAR);

        telemetry.addData("Say", "Hello Driver");
        runtime.reset();

        int[] clicksArr = getClicks(0, 180);
        distanceClicks = clicksArr[0];
        rotClicks = clicksArr[1];

        drive(distanceClicks, rotClicks);
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
        for(int i = 0; i < 4; i++){
            posData[i] = posSystem.getPositionArr()[i];
        }
        telemetry.addData("Xpos", posData[0]);
        telemetry.addData("Ypos", posData[1]);
        telemetry.addData("W", posData[2]);
        telemetry.addData("R", posData[3]);
        telemetry.addData("DriveType, ", posSystem.getDriveType());
        telemetry.addData("Target Rot", rotClicks);
        telemetry.addData("Target Distance", distanceClicks);
        telemetry.addData("Working?", works());
        telemetry.addData("HashPos TopR", posSystem.motorClicksPose.get("topR"));
        telemetry.addData("HashPosPrev TopR", posSystem.prevMotorClicks.get("topR"));
        telemetry.addData("HashPos BotR", posSystem.motorClicksPose.get("botR"));
        telemetry.addData("HashPosPrev BotR", posSystem.prevMotorClicks.get("botR"));
        telemetry.addData("topR isBusy", robot.topR.isBusy());
        telemetry.addData("botR isBusy", robot.botR.isBusy());
        telemetry.update();
    }


    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }

    void DriveTrainBasePower(){
//        int powerBotL = 1;
//        int powerTopL = 1;
//
//        if (gamepad1.dpad_up){
//            robot.botL.setPower(powerBotL);
//            robot.topL.setPower(powerTopL);
//        }
//        else{
//            robot.botL.setPower(0);
//            robot.topL.setPower(0);
//        }
    }

    void DriveTrainPowerEncoder(){
        robot.botL.setPower(0.2);
        robot.topL.setPower(0.2);
        robot.botR.setPower(0.2);
        robot.topR.setPower(0.2);

        posSystem.calculatePos();
    }

    public void drive(int distanceClicks, int rotClicks){
        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() - distanceClicks + rotClicks);
        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + distanceClicks + rotClicks);
        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() - distanceClicks + rotClicks);
        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + distanceClicks + rotClicks);

        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int[] getClicks(double distance, double rotation){
        int[] clicks = new int[2];
        double translationClicks = distance * constants.CLICKS_PER_INCH; //rotation clicks
        double rotationClicks = rotation * constants.CLICKS_PER_DEGREE; //table spinning clicks

        clicks[0] = (int)translationClicks;
        clicks[1] = (int)rotationClicks;
        return clicks;
    }

    public boolean works(){
        return (posSystem.dChange() && !posSystem.rChange() && !posSystem.wChange());
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