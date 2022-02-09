package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "carousel test")
public class CarouselMotionProfile extends LinearOpMode {

    public static int accelerate1;
    public static int accelerate2;
    public static int accelerate3;
    public DcMotorEx carousel;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        carousel = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        //carousel.setVelocityPIDFCoefficients(0,0,0,0);

        int encoderPos;
        int goal = 1300;
        double velocity;
        boolean carouselOn = false;

//        MotionProfile profile1 = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(0, 0, 0),
//                new MotionState(goal, 0, 0),
//                750,
//                500
//        );
//        MotionProfile profile2 = MotionProfileGenerator.generateSimpleMotionProfile(
//                new MotionState(60, 25, 0),
//                new MotionState(120, 40, 20),
//                40,
//                20,
//                100
//        );
//        MotionProfileBuilder motionProfileBuilder= new MotionProfileBuilder(new MotionState(0,0,0));
//        motionProfileBuilder.appendProfile(profile1);
//                //.appendProfile(profile2);
//        MotionProfile profile= motionProfileBuilder.build();

//        ElapsedTime elapsedTime = new ElapsedTime();
//        carousel.getController();
//        PIDFController controller = new PIDFController(new PIDCoefficients(0, 0, 0));
        waitForStart();

        while (opModeIsActive()) {
            if (carouselOn) {
////                MotionState state = profile.get(elapsedTime.milliseconds() / 1000);
//                telemetry.addData("x", state.getX());
//                telemetry.addData("v", state.getV());
                if (state.getX() < goal) {
                    carousel.setVelocity(Math.max(800, state.getV() * 4));
                } else {
                    carouselOn = false;
                    carousel.setVelocity(0);
                }

//                controller.setTargetPosition(state.getX());
//                controller.setTargetVelocity(state.getV());
//                controller.setTargetAcceleration(state.getA());


//                if (state.getX()>=600){
//                    carouselOn= false;
//                    carousel.setVelocity(0);
//                }
            }

            if (gamepad1.a) {
                carouselOn = true;
                telemetry.addData("carousel on", "1");
            } else if (gamepad1.x) {
                carouselOn = false;
                carousel.setVelocity(0);
            }

            telemetry.update();
        }

    }
}
