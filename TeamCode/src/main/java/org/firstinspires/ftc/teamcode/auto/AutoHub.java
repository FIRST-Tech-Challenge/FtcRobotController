//package org.firstinspires.ftc.teamcode.auto;
//
//import android.app.Activity;
//import android.view.View;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.common.Constants;
//import org.firstinspires.ftc.teamcode.common.HardwareDrive;
//import org.firstinspires.ftc.teamcode.common.Kinematics.AutoKinematics;
//import org.firstinspires.ftc.teamcode.common.Kinematics.Kinematics;
//import org.firstinspires.ftc.teamcode.common.gps.GlobalPosSystem;
//
//public class AutoHub {
//    LinearOpMode linearOpMode;
//    HardwareDrive robot;
//    HardwareMap hardwareMap;
//    Constants constants = new Constants();
//    AutoKinematics kinematics;
//    GlobalPosSystem posSystem;
//
//    View relativeLayout;
//
//    public AutoHub(LinearOpMode plinear){
//        linearOpMode = plinear;
//        hardwareMap = linearOpMode.hardwareMap;
//        robot = new HardwareDrive();
//        robot.init(hardwareMap);
//
//        // Send telemetry message to signify robot waiting;
//        linearOpMode.telemetry.addData("Status", "Resetting Encoders and Camera");
//        linearOpMode.telemetry.update();
//
//        posSystem = new GlobalPosSystem(robot);
//        kinematics = new AutoKinematics(posSystem);
//        posSystem.grabKinematics(kinematics);
//
//        // Get a reference to the RelativeLayout so we can later change the background
//        // color of the Robot Controller app to match the hue detected by the RGB sensor.
//        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
//        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
//
//        robot.setRunMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        linearOpMode.telemetry.addData("Status", "Waiting on Camera");
//        linearOpMode.telemetry.update();
//
//    }
//
//
//    public void linearMovement(double x, double y, double turnDegrees, double speed, double kp, double ki, double kd){
//        posSystem.calculatePos();
//        kinematics.setCurrents();
//
//        //snap
//        kinematics.setPos(Kinematics.DriveType.SNAP, x, y, 0, 1);
//        while (linearOpMode.opModeIsActive() && kinematics.shouldSnap()){
//            posSystem.calculatePos();
//            kinematics.setCurrents();
//            kinematics.logic();
//            robot.dtMotors[0].setPower(kinematics.getPower()[0]);
//            robot.dtMotors[1].setPower(-kinematics.getPower()[1]);
//            robot.dtMotors[2].setPower(kinematics.getPower()[2]);
//            robot.dtMotors[3].setPower(-kinematics.getPower()[3]);
//        }
//        robot.setMotorPower(0); //this might be unnecessary
//
//
//        //move
//        kinematics.setPos(Kinematics.DriveType.LINEAR, x, y, turnDegrees, speed);
//
//        int[] encoderTargets = new int[4];
//        for (int i = 0; i < 4; i++){
//            encoderTargets[i] = kinematics.getLinearClicks()[i];
//            robot.dtMotors[i].setTargetPosition(encoderTargets[i]);
//        }
//        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (linearOpMode.opModeIsActive() && robot.wheelsAreBusy()){
//            posSystem.calculatePos();
//            kinematics.setCurrents();
//            kinematics.logic();
//            for (int i = 0; i < 3; i++){
//                robot.dtMotors[i].setVelocity(kinematics.getVelocity()[i]);
//            }
//        }
//        robot.setMotorPower(0);
//        //reset()
//        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void spline(double x, double y, double turnDegrees, double speed, double kp, double ki, double kd){
//        posSystem.calculatePos();
//        kinematics.setCurrents();
//
//        kinematics.setPos(Kinematics.DriveType.SPLINE, x, y, turnDegrees, speed);
//
//        int targets[] = new int[4];
//        for (int i = 0; i < 4; i++){
//            targets[i] = kinematics.getSplineClicks()[i];
//            robot.dtMotors[i].setTargetPosition(targets[i]);
//        }
//
//        robot.setRunMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        while (linearOpMode.opModeIsActive() && robot.wheelsAreBusy()){
//            posSystem.calculatePos();
//            kinematics.setCurrents();
//            kinematics.logic();
//            for (int i = 0; i < 3; i++){
//                robot.dtMotors[i].setVelocity(kinematics.getVelocity()[i]);
//            }
//        }
//        robot.setMotorPower(0);
//        //reset()
//        robot.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
//    }
//
//    public void turn(double turnDegrees, double speed, double kp, double ki, double kd){
//
//    }
//}
