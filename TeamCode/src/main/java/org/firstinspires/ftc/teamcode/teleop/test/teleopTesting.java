package org.firstinspires.ftc.teamcode.teleop.test;

import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;
import org.firstinspires.ftc.teamcode.common.HardwareDrive;
import org.firstinspires.ftc.teamcode.common.Kinematics.TeleopKinematics;
import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
import org.firstinspires.ftc.teamcode.common.pid.RotateSwerveModulePID;

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

        testMove(0,1);
        telemetry.update();
    }


    void testMove(double r, double s){
        double lx = gamepad1.left_stick_x;
        double ly = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        double ry = gamepad1.right_stick_y;

//        double rotatePower = r;
//        double spinPower = s;
//        double translationPowerPercentage = 0.5;
//        double rotationPowerPercentage = 0.5;
//        double leftThrottle = 1.0;
//        double rightThrottle = 1.0;
//        int rotationSwitchMotors = 1; //1 if rotating wheels right, -1 if rotating wheels left
//        int translateSwitchMotors = 1; //1 if going forward, -1 if going backward


        double power = Math.sqrt(Math.pow(lx, 2) + Math.pow(ly, 2));

        double[] rotateModule = wheelOptimization(lx, ly, 0);
        double rotateClicks = rotateModule[0]; //degrees
        rotateClicks *= constants.CLICKS_PER_DEGREE; //clicks
        double turnDirection = rotateModule[2];

        double spinClicks = power * constants.MAX_VELOCITY_DT * constants.LOOP_ITERATION_TIME; // clicks

        robot.topL.setTargetPosition((int)spinClicks);
        robot.botL.setTargetPosition(-(int)spinClicks);

        robot.topL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.botL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.topL.setPower(power);
        robot.topL.setPower(-power);

        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public double[] wheelOptimization(double x, double y, double currentW){ //returns how much the wheels should rotate in which direction
        double[] directionArr = new double[3];

        //determine targets
        double target = (y==0 ? (90 * Math.signum(x)) : Math.toDegrees(Math.atan2(x, y)));
        directionArr[1] = target;

        //determine how much modules must turn in which direction (optimization)
        double turnAmount = target - currentW;
        double turnDirection = Math.signum(turnAmount);

        if(Math.abs(turnAmount) > 180){
            turnAmount = 360 - Math.abs(turnAmount);
            turnDirection *= -1;
        }

        if(Math.abs(turnAmount) > 90){
            target += 180;
            target = clamp(target);
            turnAmount = target - currentW;
            turnDirection *= -1;
            if(Math.abs(turnAmount) > 180){
                turnAmount = 360 - Math.abs(turnAmount);
            }
        }
        directionArr[0] = Math.abs(turnAmount);
        directionArr[2] = turnDirection;

        return directionArr;
    }

    public double clamp(double degrees){
        if (Math.abs(degrees) >= 360) degrees %= 360;

        if (degrees < -179 || degrees > 180) {
            int modulo = (int)Math.signum(degrees) * -180;
            degrees = Math.floorMod((int)degrees, modulo);
        }
        return degrees;
    }

    @Override
    public void stop() {
//        telemetry.addData("X", posSystem.getPositionArr()[0]);
//        telemetry.addData("Y", posSystem.getPositionArr()[1]);
//        telemetry.addData("W", posSystem.getPositionArr()[2]);
//        telemetry.addData("R", posSystem.getPositionArr()[3]);
//        telemetry.update();
        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
}
