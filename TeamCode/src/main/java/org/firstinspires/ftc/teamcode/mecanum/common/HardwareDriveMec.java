package org.firstinspires.ftc.teamcode.mecanum.common;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.kinematics.ArmKinematics;
import org.firstinspires.ftc.teamcode.common.constantsPKG.Constants;

@TeleOp(name = "Hardware Drive Mech", group = "Drive")

public class HardwareDriveMec extends OpMode{
    //local class objects
    HardwareDrive robot = new HardwareDrive();
    Constants constants = new Constants();

    //buttons
    Button x = new Button();
    Button y = new Button();
    Button a = new Button();
    Button b = new Button();

    ElapsedTime resetTimer = new ElapsedTime();
    View relativeLayout;

    int prevPosition;

    private double power = 0.5;

    @Override
    public void init(){
        robot.init(hardwareMap);

        telemetry.addData("Say", "Hello Driver");
    }

    @Override
    public void init_loop() { //Loop between "init" and "start"
        //  robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        robot.topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.botL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void start(){

    }

    @Override
    public void loop() { //Loop between "start" and "stop"
        UpdatePlayer1();
        UpdateButton();
        UpdateTelemetry();
    }

    private void UpdateTelemetry() {
        telemetry.addData("Top R", robot.topR.getCurrentPosition());
        telemetry.addData("Top L", robot.topL.getCurrentPosition());
        telemetry.addData("Bot R", robot.botR.getCurrentPosition());
        telemetry.addData("Bot L", robot.botL.getCurrentPosition());
        telemetry.addData("Power", power);
        telemetry.update();
    }

    private void UpdateButton() {
        x.update(gamepad1.x);
        y.update(gamepad1.y);
        a.update(gamepad1.a);
        b.update(gamepad1.b);
    }

    void UpdatePlayer1(){
        Drive();
    }

    private void Drive(){
        double directionX = Math.pow(gamepad1.left_stick_x, 1); //Strafe
        double directionY = -Math.pow(gamepad1.left_stick_y, 1); //Forward
        double directionR = Math.pow(gamepad1.right_stick_x, 1); //Turn


        robot.topL.setPower((directionY + directionR + directionX) * DriveTrainSpeed());
        robot.topR.setPower((directionY - directionR - directionX) * DriveTrainSpeed());
        robot.botL.setPower((directionY + directionR - directionX) * DriveTrainSpeed());
        robot.botR.setPower((directionY - directionR + directionX) * DriveTrainSpeed());
    }

    private double DriveTrainSpeed(){
        double drivePower = 0.75;

        if (gamepad1.right_bumper)
            drivePower = 1;
        else if (gamepad1.left_bumper)
            drivePower = 0.25;
        
        return drivePower;
    }


    @Override
    public void stop() {
    }
}
