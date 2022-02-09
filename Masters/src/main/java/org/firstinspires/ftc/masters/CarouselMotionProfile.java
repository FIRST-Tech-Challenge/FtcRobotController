package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
@TeleOp(name = "carousel test quick duck")
public class CarouselMotionProfile extends LinearOpMode {

    public static int accelerate1 = 300;
    public static int accelerate2= 200;
    public static int accelerate3 = 600;
    public static  int startVelocity= 650;
    public DcMotorEx carousel;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = dashboard.getTelemetry();

        carousel = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setVelocityPIDFCoefficients(10,0,0.01,14);

        int encoderPos=0;
        int goal = 3500;

        double velocity=0;
        boolean carouselOn = false;
        boolean start = true;

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

       ElapsedTime elapsedTime = new ElapsedTime();
//        carousel.getController();
//        PIDFController controller = new PIDFController(new PIDCoefficients(0, 0, 0));
        waitForStart();
        double vel2Max=0;
        double vel1Max=0;

        while (opModeIsActive()) {
            //telemetry.addData("carousel test", "1");
            telemetry.addData("velocity", carousel.getVelocity());
            telemetry.addData("encoder", carousel.getCurrentPosition());
            telemetry.addData("time", elapsedTime.milliseconds());
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

            if (gamepad1.a) {
                carouselOn = true;
                if (start) {
                    elapsedTime.reset();
                    start= false;
                }

            } else if (gamepad1.x) {
                carouselOn = false;
                carousel.setVelocity(0);

            }

            if (carouselOn) {

                encoderPos = carousel.getCurrentPosition();

                if (encoderPos < 600) {
                    velocity = Math.sqrt(2*accelerate1*encoderPos)+startVelocity;
                    vel1Max = velocity;
                    carousel.setVelocity(velocity);
                    telemetry.update();
                } else if (encoderPos >= 500 && encoderPos < 1900) {
                    velocity = vel1Max + Math.sqrt(2 * accelerate2 * (encoderPos - 600));
                    vel2Max = velocity;
                    carousel.setVelocity(velocity);
                    telemetry.update();
                }
                 else if (encoderPos >= 1900 && encoderPos < goal) {
                    velocity = vel2Max+Math.sqrt(2*accelerate3*(encoderPos-1600));
                    carousel.setVelocity(velocity);
                    telemetry.update();
                } else if (encoderPos >= goal) {
                    carouselOn = false;
                    carousel.setVelocity(0);
                    carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                }

            }




        }

    }
}
