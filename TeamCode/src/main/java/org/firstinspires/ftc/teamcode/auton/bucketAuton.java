package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;
import org.firstinspires.ftc.teamcode.myUtil.threads.teleOp.lineUp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.compVis;
import org.firstinspires.ftc.teamcode.myUtil.computerVision.visionLib;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.myUtil.Hardware;
import org.firstinspires.ftc.teamcode.myUtil.MecanumHardAuto;

@Autonomous(name="bucket auton")
public class bucketAuton extends LinearOpMode {
    MecanumHardAuto r = new MecanumHardAuto();
    @Override
    public void runOpMode() throws InterruptedException {
        r.initRobot(this);

        // wrist has max at

        //4
        telemetry.update();
        waitForStart();

        //first bucket
        r.setClaw(0);
        r.moveInches(0.5,5);
        r.superRotate(0.5,45, Hardware.directions.LEFT);
        r.moveInches(0.5,10);
        r.superRotate(0.5,45, Hardware.directions.LEFT);
        r.moveInches(0.5,17);
        r.setArm(0.8,60);
        r.setLinearSlide(0.8,18);
        r.waiter(200);
        r.setArm(0.8,55);
        r.setClaw(1);
        r.waiter(200);
        r.arm.setDirection(DcMotorSimple.Direction.REVERSE);
        r.setArm(0.8,60);
        r.arm.setDirection(DcMotorSimple.Direction.FORWARD);
        r.linear_slide.setDirection(DcMotorSimple.Direction.REVERSE);
        r.setLinearSlide(0.8,18);
        r.linear_slide.setDirection(DcMotorSimple.Direction.FORWARD);





    }
}
