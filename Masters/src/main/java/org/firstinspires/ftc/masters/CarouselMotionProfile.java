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

    public static int accelerate1 = 600;
    public static int accelerate2= 65;
    public static int accelerate3 = 300;
    public static  int startVelocity= 620;
    public static int region1 = 750;
    public static int region2 = 2000;
    public DcMotorEx carousel;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void runOpMode() throws InterruptedException {


        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        carousel = hardwareMap.get(DcMotorEx.class, "carouselMotor");
        carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        carousel.setVelocityPIDFCoefficients(10,0,0.01,14);

        int encoderPos=0;
        double velocity=0;
        boolean carouselOn = false;
        boolean start = true;

       ElapsedTime elapsedTime = new ElapsedTime();

        waitForStart();
        double vel2Max=0;
        double vel1Max=0;

        while (opModeIsActive()) {
            //telemetry.addData("carousel test", "1");
            telemetry.addData("velocity", carousel.getVelocity());
            telemetry.addData("encoder", carousel.getCurrentPosition());
            telemetry.addData("time", elapsedTime.milliseconds());


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

                if (encoderPos < region1) {
                    velocity = Math.sqrt(2*accelerate1*encoderPos)+startVelocity;
                    vel1Max = velocity;
                    carousel.setVelocity(velocity);
                    telemetry.update();
                } else if (encoderPos >= region1 && encoderPos < region2) {
                    velocity = vel1Max + Math.sqrt(2 * accelerate2 * (encoderPos - region1));
                    vel2Max = velocity;
                    carousel.setVelocity(velocity);
                    telemetry.update();
                }
                 else if (encoderPos >= region2 && encoderPos < FreightFrenzyConstants.goal) {
                    velocity = vel2Max+Math.sqrt(2*accelerate3*(encoderPos-region2));
                    carousel.setVelocity(velocity);
                    telemetry.update();
                } else if (encoderPos >= FreightFrenzyConstants.goal) {
                    carouselOn = false;
                    carousel.setVelocity(0);
                    carousel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    start = true;

                }

            }




        }

    }
}
