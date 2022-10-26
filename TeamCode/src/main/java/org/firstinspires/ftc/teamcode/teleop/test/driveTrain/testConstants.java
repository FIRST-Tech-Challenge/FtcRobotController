package org.firstinspires.ftc.teamcode.teleop.test.driveTrain;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;
import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.Button;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;

@TeleOp(name="test constants", group="Drive")
//@Disabled
public class testConstants extends OpMode{
    /* Declare OpMode members. */
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();

    private ElapsedTime runtime = new ElapsedTime();

    public enum State{
        SAME_DIRECTION,
        SINGLE,
        NOT_INITIALIZED
    }
    public State state = State.NOT_INITIALIZED;


    public enum TestConstant{
        SPIN,
        ROTATE,
        NOT_INITIALIZED
    }
    public TestConstant testConstant = TestConstant.NOT_INITIALIZED;

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
        telemetry.addData("State", state);
        telemetry.addData("Test Type", testConstant);
        telemetry.addData("TopR", robot.topR.getCurrentPosition());
        telemetry.addData("TopL", robot.topL.getCurrentPosition());

        telemetry.update();
    }

    void UpdateButton(){
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }

    void DriveTrainPowerEncoder(){
        double beta = 0.5;
        double alpha = 1 - beta;

        if (a.getState() == Button.State.TAP) state = State.SAME_DIRECTION;
        else if (b.getState() == Button.State.TAP) state = State.SINGLE;

        if (x.getState() == Button.State.TAP) testConstant = TestConstant.SPIN;
        else if (y.getState() == Button.State.TAP) testConstant = TestConstant.ROTATE;

        int distanceTopR = 0;
        int distanceBotR = 0;

        int rotationalTopR = 0;
        int rotationalBotR = 0;

        switch(testConstant){
            case SPIN:
                distanceTopR = (int)(constants.CLICKS_PER_INCH * constants.WHEEL_CIRCUMFERENCE);
                distanceBotR = -1 * (int)(constants.CLICKS_PER_INCH * constants.WHEEL_CIRCUMFERENCE);
                break;

            case ROTATE:
                rotationalTopR = (int)(constants.CLICKS_PER_DEGREE * 360);
                rotationalBotR = (int)(constants.CLICKS_PER_DEGREE * 360);
                break;
        }

        switch (state){
            case SAME_DIRECTION:
                robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + distanceTopR + rotationalTopR);
                robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + distanceBotR + rotationalBotR);

                robot.topR.setPower(gamepad1.left_stick_y * 0.2 + gamepad1.left_stick_x * 0.2);
                robot.botR.setPower(gamepad1.left_stick_y * 0.2 + gamepad1.left_stick_x * 0.2);
                break;

            case SINGLE:
                robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + distanceTopR + rotationalTopR);

                robot.topR.setPower(gamepad1.left_stick_y * 0.2 + gamepad1.left_stick_x * 0.2);
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