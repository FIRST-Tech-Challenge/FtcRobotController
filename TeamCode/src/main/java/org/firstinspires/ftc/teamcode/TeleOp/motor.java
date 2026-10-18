//
//
//package org.firstinspires.ftc.teamcode.TeleOp;
//
//import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//
//@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Field Centric")
//public class asd extends LinearOpMode {
//
//    private DcMotor leftFront;
//    private DcMotor rightFront;
//    private DcMotor leftBack;
//    private DcMotor rightBack;
//
//    //pinpoint
//    private GoBildaPinpointDriver odo;
//
//    @Override
//    public void runOpMode(){
//        leftFront = hardwareMap.get(DcMotor.class, "lf");
//        rightFront = hardwareMap.get(DcMotor.class, "rf");
//        leftBack = hardwareMap.get(DcMotor.class, "lb");
//        rightBack = hardwareMap.get(DcMotor.class ,"rb");
//        odo =  hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
//        odo.setOffsets(-3, -3.5, DistanceUnit.INCH);
//        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        odo.resetPosAndIMU();
//
//        leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        telemetry.addLine("it works");
//
//        waitForStart();
//        if(isStopRequested()) return;
//
//        boolean prevA = false;
//        boolean stateA = false;
//
//        while(opModeIsActive()) {
//
//            boolean padA = gamepad1.a;
////            if(padA != prevA){
////                if (padA) {
////                    stateA = true;
////                } else {
////                    stateA = false;
////                }
////            }
////            prevA = padA;
//
//
//            leftFront.setPower(lfPower);
//            leftBack.setPower(lbPower);
//            rightFront.setPower(rfPower);
//            rightBack.setPower(rbPower);
//
//            telemetry.addLine("its running");
//            telemetry.addData("stateA: ", stateA);
//            telemetry.update();
//
//            telemetry.update();
//        }
//
//
//
//
//    }
//}