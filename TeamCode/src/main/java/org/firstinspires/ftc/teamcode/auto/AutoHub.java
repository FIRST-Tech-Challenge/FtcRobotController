//package org.firstinspires.ftc.teamcode.auto;
//
//import android.app.Activity;
//import android.view.View;
//
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.common.ConstantsPKG.Constants;
//import org.firstinspires.ftc.teamcode.common.HardwareDrive;
////import org.firstinspires.ftc.teamcode.common.Kinematics.AutoKinematics;
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
//    void Move(String movementType, double x, double y, double finalAngle, double power){
//
//        switch(movementType){
//            case "linear":
////                robot.botL.setPower(0);
//
//                break;
//
//            case "throttle":
////                robot.topL.setPower(1);
//                break;
//
//            case "tableSpin":
////                robot.botR.setPower(0.5);
//                break;
//
//            case "rotate":
////                robot.topR.setPower(0.3);
//                break;
//
//
//        }
//
//
//
//
//
//    }
//}
