//package org.firstinspires.ftc.teamcode.teleop.test.driveTrain;
//
//import android.view.View;
//
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.common.Kinematics.LinearKinematicsTestJR;
//import org.firstinspires.ftc.teamcode.common.Kinematics.SplineKinematicsTest;
//import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
//import org.firstinspires.ftc.teamcode.common.Button;
//import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;
//
//import org.firstinspires.ftc.teamcode.common.HardwareDrive;
//
//@TeleOp(name="Spline Base Drive Test", group="Drive")
////@Disabled
//public class SplineBaseDrive extends OpMode{
//    public enum ControllerType{
//        CONTOLLER,
//        BUTTON,
//        NOT_INITIALIZED
//    }
//
//    public enum telemetryType{
//        GPS,
//        TARGETS
//    }
//    telemetryType tType = telemetryType.GPS;
//    ControllerType controllerType = ControllerType.NOT_INITIALIZED;
//
//    /* Declare OpMode members. */
//    HardwareDrive robot = new HardwareDrive();
//    Constants constants = new Constants();
//    GlobalPosSystem posSystem;
//    SplineKinematicsTest kinematics;
//
//    private ElapsedTime runtime = new ElapsedTime();
//
//    boolean isResetCycle = false;
//
//    Button x = new Button();
//    Button y = new Button();
//    Button a = new Button();
//    Button b = new Button();
//
//    //for resetting the robot's wheels' orientation
//    ElapsedTime resetTimer = new ElapsedTime();
//    /** The relativeLayout field is used to aid in providing interesting visual feedback
//     * in this sample application; you probably *don't* need this when you use a color sensor on your
//     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller. */
//    View relativeLayout;
//
//    @Override
//    public void init() { //When "init" is clicked
//        robot.init(hardwareMap);
//
//        telemetry.addData("Say", "Hello Driver");
//        runtime.reset();
//
//        posSystem = new GlobalPosSystem(robot);
//        kinematics = new SplineKinematicsTest(posSystem);
//        posSystem.grabKinematics(kinematics);
//    }
//
//    @Override
//    public void init_loop() { //Loop between "init" and "start"
//        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    @Override
//    public void start() { //When "start" is pressed
//
//    }
//
//    @Override
//    public void loop() { //Loop between "start" and "stop"
//        UpdatePlayer1();
//        UpdatePlayer2();
//        UpdateButton();
//        UpdateTelemetry();
//    }
//
//    void UpdatePlayer1(){
//        if (a.getState() == Button.State.TAP){
//            tType = telemetryType.GPS;
//        } else if (b.getState() == Button.State.TAP){
//            tType = telemetryType.TARGETS;
//        }
//
//        if (x.getState() == Button.State.TAP) controllerType = ControllerType.CONTOLLER;
//        else if (y.getState() == Button.State.TAP) controllerType = ControllerType.BUTTON;
//
//        if (x.getState() == Button.State.DOUBLE_TAP){
//            reset();
//        } else{
//            DriveTrainBase();
//        }
//    }
//
//    void UpdatePlayer2(){
//    }
//
//    void UpdateTelemetry(){
//        double[] posData = posSystem.getPositionArr();
//
//        if (tType == telemetryType.TARGETS){
//            telemetry.addData("X gamepad", gamepad1.left_stick_x);
//            telemetry.addData("Y gamepad", gamepad1.left_stick_y);
//            telemetry.addData("Left W", posData[2]);
//            telemetry.addData("Right W", posData[3]);
//            telemetry.addData("Right TargetW", kinematics.getRTargetW());
//            telemetry.addData("Left TargetW", kinematics.getLTargetW());
//            telemetry.addData("Robot", posSystem.getPositionArr()[4]);
//
//            telemetry.addData("Right Turn Amount", kinematics.getRTurnAmount());
//            telemetry.addData("Left Turn Amount", kinematics.getLTurnAmount());
//
//            telemetry.addData("Right Optimized Target", kinematics.getROptimizedTargetW());
//            telemetry.addData("Left Optimized Target", kinematics.getLOptimizedTargetW());
//
//            telemetry.addData("Right Direction", kinematics.getRightDirectionW());
//            telemetry.addData("Left Direction", kinematics.getLeftDirectionW());
//
//            telemetry.addData("Should Snap?", kinematics.shouldSnap());
//
//        } else if (tType == telemetryType.GPS){
//            telemetry.addData("X", posData[0]);
//            telemetry.addData("Y", posData[0]);
//            telemetry.addData("Left W", posData[2]);
//            telemetry.addData("Right W", posData[3]);
//            telemetry.addData("R", posData[4]);
//            telemetry.addData("Drive Type", kinematics.getdDriveType());
//            telemetry.addData("Power Top", kinematics.getPower()[0]);
//            telemetry.addData("Power Bottom", kinematics.getPower()[1]);
//            telemetry.addData("Rot Clicks", kinematics.rightRotClicks);
//            telemetry.addData("Spin clicks", kinematics.spinClicks);
//            telemetry.addData("topL Clicks", robot.topL.getCurrentPosition());
//            telemetry.addData("botL Clicks", robot.botL.getCurrentPosition());
//        }
//
//        telemetry.update();
//    }
//
//    void UpdateButton(){
//        x.update(gamepad1.x);
//        y.update(gamepad1.y);
//        a.update(gamepad1.a);
//        b.update(gamepad1.b);
//    }
//
//    void DriveTrainBase(){
//        DriveTrainMove();
//    }
//
//    private void DriveTrainMove(){
//        //gps system
//        posSystem.calculatePos();
//        kinematics.setCurrents();
//
//        //setting targets
//        setVariables();
//
//        //put power into the motors
//        setPower();
//
//    }
//
//    private void setVariables(){
//        //outputs of joysticks
//        double left_stick_x = gamepad1.left_stick_x; //returns a value between [-1, 1]
//        double left_stick_y = -gamepad1.left_stick_y; //returns a value between [-1, 1]
//        double right_stick_x = gamepad1.right_stick_x; //returns a value between [-1, 1]
//        double right_stick_y = -gamepad1.right_stick_y; //returns a value between [-1, 1]
//
//        switch(controllerType){
//            case CONTOLLER:
//                kinematics.getGamepad(left_stick_x, left_stick_y, right_stick_x, right_stick_y);
//
//                kinematics.setPos();
//
//                kinematics.logic();
//                break;
//
//            case BUTTON:
//                kinematics.getGamepad(-1, 0, 0, 0);
//
//                kinematics.setPos();
//
//                kinematics.logic();
//                break;
//        }
//    }
//
//
//    private void setPower(){
//        double[] motorPower = kinematics.getPower();
//
//        if (motorPower[0] == 0 && motorPower[1] == 0 && motorPower[2] == 0 && motorPower[3] == 0){
//            robot.setMotorPower(0);
//            return;
//        }
//
//        int[] targetClicks = kinematics.getClicks();
//        robot.topL.setTargetPosition(robot.topL.getCurrentPosition() + targetClicks[0]);
//        robot.botL.setTargetPosition(robot.botL.getCurrentPosition() + targetClicks[1]);
//        robot.topR.setTargetPosition(robot.topR.getCurrentPosition() + targetClicks[2]);
//        robot.botR.setTargetPosition(robot.botR.getCurrentPosition() + targetClicks[3]);
//
//        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        robot.topL.setPower(motorPower[0] * constants.POWER_LIMITER);
//        robot.botL.setPower(motorPower[1] * constants.POWER_LIMITER);
//        robot.topR.setPower(motorPower[2] * constants.POWER_LIMITER);
//        robot.botR.setPower(motorPower[3] * constants.POWER_LIMITER);
//
////        if (motorPower[0] == 0) robot.topL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        if (motorPower[1] == 0) robot.botL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        if (motorPower[2] == 0) robot.topR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        if (motorPower[3] == 0) robot.botR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//    }
//
//    private void reset(){
//        if (isResetCycle && !robot.wheelsAreBusy()){
//            robot.topL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.botL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.topR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            robot.botR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            robot.topL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.botL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.topR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            robot.botR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//            robot.botL.setPower(0);
//            robot.topL.setPower(0);
//            robot.botR.setPower(0);
//            robot.topR.setPower(0);
//        }
//
//        if (!isResetCycle){
//            isResetCycle = true;
//            int topR = robot.topR.getCurrentPosition();
//            int botR = robot.botR.getCurrentPosition();
//            int topL = robot.topL.getCurrentPosition();
//            int botL = robot.botL.getCurrentPosition();
//
//            int rotateR = (topR + botR) / 2;
//            int rotateL = (topL + botL) / 2;
//
//            int topLTarget = (int)((topL - rotateL) % constants.CLICKS_PER_PURPLE_REV);
//            int botLTarget = (int)((botL - rotateL) % constants.CLICKS_PER_PURPLE_REV);
//            int topRTarget = (int)((topR - rotateR) % constants.CLICKS_PER_PURPLE_REV);
//            int botRTarget = (int)((botR - rotateR) % constants.CLICKS_PER_PURPLE_REV);
//
//            robot.topL.setTargetPosition(topLTarget);
//            robot.botL.setTargetPosition(botLTarget);
//            robot.topR.setTargetPosition(topRTarget);
//            robot.botR.setTargetPosition(botRTarget);
//
//            robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.botR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            robot.topR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        } else{
//            robot.botL.setPower(0.2);
//            robot.topL.setPower(0.2);
//            robot.botR.setPower(0.2);
//            robot.topR.setPower(0.2);
//        }
//        //make sure to reset the encoder position afterwards without messing stuff up like before.
//    }
//
//    /*
//     * Code to run ONCE after the driver hits STOP
//     */
//    @Override
//    public void stop() {
//    }
//}