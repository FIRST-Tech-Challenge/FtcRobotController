package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Evolution Training Test", group="Evolution")
//@Disabled
public class EvolutionTrainingTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime launchertime = new ElapsedTime();
    private ElapsedTime xButtonDelay = new ElapsedTime();

    private DcMotorEx leftDriveFront = null;
    private DcMotorEx rightDriveFront = null;
    private DcMotorEx leftDriveBack = null;
    private DcMotorEx rightDriveBack = null;

    private DcMotor planeLauncher = null;
    private Servo planeLoader = null;

    private DcMotor ziptie = null;
    private DcMotor lift = null;
    private Servo claw0 = null;
    private Servo claw1 = null;
    private boolean clawO = true;
    double clawOpen;
    private int liftPos = 0;

    private DcMotor lift2 = null;

    WolfNet[] networks;
    WolfNet[] nextGen;

    double[] in;
    double[] out;
    double[] errors;

    int updates = 0;
    boolean driving;

    @Override
    public void runOpMode() {
        leftDriveFront = hardwareMap.get(DcMotorEx.class, "left_drive_front");
        rightDriveFront = hardwareMap.get(DcMotorEx.class, "right_drive_front");
        leftDriveBack  = hardwareMap.get(DcMotorEx.class, "left_drive_back");
        rightDriveBack = hardwareMap.get(DcMotorEx.class, "right_drive_back");

        planeLauncher = hardwareMap.get(DcMotor.class, "airplane");
        planeLoader = hardwareMap.get(Servo.class, "plain_loader");

        ziptie = hardwareMap.get(DcMotor.class, "ziptie");
        lift = hardwareMap.get(DcMotor.class, "lift");
        claw0 = hardwareMap.get(Servo.class, "claw0");
        claw1 = hardwareMap.get(Servo.class, "claw1");

        lift2 = hardwareMap.get(DcMotor.class, "spider_man");

        leftDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDriveBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDriveBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ziptie.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftDriveFront.setDirection(DcMotor.Direction.REVERSE);
        rightDriveFront.setDirection(DcMotor.Direction.REVERSE);
        leftDriveBack.setDirection(DcMotor.Direction.FORWARD);
        rightDriveBack.setDirection(DcMotor.Direction.FORWARD);

        planeLauncher.setDirection(DcMotor.Direction.REVERSE);

        ziptie.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        lift2.setDirection(DcMotor.Direction.FORWARD);

        claw0.setPosition(1.0f);
        claw1.setPosition(0.0f);

        networks = new WolfNet[50];
        nextGen = new WolfNet[5];

        errors = new double[50];

        driving = true;

        for(int i = 0; i < 5; i++){
            nextGen[i] = new WolfNet(8, 6, 2, "Weights" + i, 0.5);
            nextGen[i].LoadWeights();
        }
        for(int i = 0; i < 50; i++){
            if(i < 5){
                networks[i] = new WolfNet(nextGen[i], "Weights");
            }else if(i < 14){
                networks[i] = new WolfNet(nextGen[0], "Weights");
                networks[i].MutateWeights();
            }else if(i < 23){
                networks[i] = new WolfNet(nextGen[1], "Weights");
                networks[i].MutateWeights();
            }else if(i < 32){
                networks[i] = new WolfNet(nextGen[2], "Weights");
                networks[i].MutateWeights();
            }else if(i < 41){
                networks[i] = new WolfNet(nextGen[3], "Weights");
                networks[i].MutateWeights();
            }else {
                networks[i] = new WolfNet(nextGen[4], "Weights");
                networks[i].MutateWeights();
            }
        }

        waitForStart();
        runtime.reset();
        xButtonDelay.reset();

        while (opModeIsActive()) {

            drive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            if(gamepad1.a) {
                planeLauncher.setPower(0.5f);
                if(launchertime.time() > 1){
                    planeLoader.setPosition(0.25f);
                }
            }else{
                planeLauncher.setPower(0.0f);
                planeLoader.setPosition(1.0f);
                launchertime.reset();
            }

            ziptie.setPower(gamepad1.right_trigger - gamepad1.left_trigger);

            if(gamepad1.left_bumper){
                lift.setPower(1.0f);
            }else if(gamepad1.right_bumper){
                lift.setPower(-1.0f);
            }else{
                lift.setPower(0.0f);
            }

            if(gamepad1.x && xButtonDelay.time() > 0.2) {
                clawO = !clawO;
                xButtonDelay.reset();
            }
            if(clawO) {
                claw0.setPosition(1.0f);
                claw1.setPosition(0.0f);
                clawOpen = 1;
            }else{
                claw0.setPosition(0.25f);
                claw1.setPosition(0.75f);
                clawOpen = -1;
            }

            if(gamepad1.dpad_up){
                lift2.setPower(1.0f);
            }else if(gamepad1.dpad_down){
                lift2.setPower(-1.0f);
            }else{
                lift2.setPower(0.0f);
            }

            updates += 1;

            for(int i = 0; i < 50; i++){
                double[][] input = {{runtime.time(), leftDriveFront.getVelocity(), rightDriveFront.getVelocity(), leftDriveBack.getVelocity(), rightDriveBack.getVelocity(), clawOpen, lift.getPower(), ziptie.getPower()}};
                double[][] correct = {{gamepad1.left_stick_x, gamepad1.right_stick_x, gamepad1.left_stick_y, clawOpen, lift.getPower(), ziptie.getPower()}};
                networks[i].GetOutput(input);
                double[][] errorI = Matrix.error(networks[i].output.layer, correct);

                double errorAvg = 0;
                for(int j = 0; j < errorI.length; j++){
                    errorAvg += errorI[j][1];
                }
                errorAvg /= errorI.length;

                errors[i] += errorAvg;
            }

            if(gamepad1.start)
                endGeneration();
        }
    }

    void drive(float x1, float x2, float y1){
        double fl = 0.0;
        double fr = 0.0;
        double bl = 0.0;
        double br = 0.0;

        fl += y1;
        fr += y1;
        bl += y1;
        br += y1;

        fl -= x1;
        fr += x1;
        bl += x1;
        br -= x1;

        fl -= x2;
        fr += x2;
        bl -= x2;
        br += x2;

        leftDriveFront.setVelocity(fl * 1000);
        rightDriveFront.setVelocity(fr * 1000);
        leftDriveBack.setVelocity(bl * 1000);
        rightDriveBack.setVelocity(br * 1000);
    }

    void endGeneration(){
        int[] bestErrors = new int[5];

        for(int i = 0; i < errors.length; i++){
            errors[i] /= updates;
        }

        for(int i = 0; i < errors.length; i++){
            for(int j = 0; j < bestErrors.length; j++){
                if(errors[i] < errors[bestErrors[j]]){
                    bestErrors[j] = i;
                    j = bestErrors.length;
                }
            }
        }

        for(int i = 0; i < bestErrors.length; i++){
            nextGen[i] = new WolfNet(networks[bestErrors[i]], "Weights" + i);
        }

        for(int i = 0; i < nextGen.length; i++){
            nextGen[i].SaveWeights();
        }

        requestOpModeStop();
    }

    double[] append(double[] array, double num){
        double[] cell = new double[array.length + 1];

        for(int i = 0; i < array.length; i++){
            cell[i] = array[i];
        }
        cell[cell.length] = num;

        return cell;
    }


}