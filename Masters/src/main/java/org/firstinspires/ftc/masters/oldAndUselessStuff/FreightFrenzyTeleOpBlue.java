//package org.firstinspires.ftc.masters.oldAndUselessStuff;
//
//
//import com.acmerobotics.roadrunner.geometry.Pose2d;
//import com.acmerobotics.roadrunner.geometry.Vector2d;
//import com.acmerobotics.roadrunner.trajectory.Trajectory;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotor;
//
//import org.firstinspires.ftc.masters.FreightFrenzyConstants;
//
//
//@TeleOp(name="freightFrenzy Blue", group = "competition")
//public class FreightFrenzyTeleOpBlue extends FreightFrenzyTeleOpRed{
//
//    public static int region1 = 1200;
//    public static int region2 = 2100;
//
//    public FreightFrenzyTeleOpBlue(){
//        super();
//        strafeConstant = -1;
//    }
//
//
//    protected void rotateCarousel(){
//        if (gamepad2.y && !carouselPushed)  {
//            if (carouselOn){
//                carouselOn = false;
//                carouselMotor.setVelocity(0);
//            } else {
//                carouselOn = true;
//                encoderCorrection = carouselMotor.getCurrentPosition();
//                carouselMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            }
//            carouselPushed= true;
//
//        } else if (!gamepad2.y) {
//            carouselPushed = false;
//        }
//
//        if (carouselOn) {
//
//            encoderPos = carouselMotor.getCurrentPosition()-encoderCorrection;
//
//            if (encoderPos > -region1) {
////                velocity = Math.sqrt(2* FreightFrenzyConstants.accelerate1*-encoderPos)+ getObject().startVelocity;
//                vel1Max = velocity;
//                carouselMotor.setVelocity(-velocity);
//                telemetry.update();
//            } else if (encoderPos <= -region1 && encoderPos > -region2) {
//                velocity = vel1Max + Math.sqrt(2 * FreightFrenzyConstants.accelerate2 * (-encoderPos - region1));
//                vel2Max = velocity;
//                carouselMotor.setVelocity(-velocity);
//                telemetry.update();
//            }
//            else if (encoderPos <= -region2 && encoderPos > -FreightFrenzyConstants.goal) {
//                velocity = vel2Max+Math.sqrt(2*FreightFrenzyConstants.accelerate3*(-encoderPos-region2));
//                carouselMotor.setVelocity(-velocity);
//                telemetry.update();
//            } else if (encoderPos <=- FreightFrenzyConstants.goal) {
//                carouselOn = false;
//                carouselMotor.setVelocity(0);
//                carouselMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//            }
//        }
//
//    }
//
////    private Object getObject() {
////        return FreightFrenzyConstants;
////    }
//
//    protected Trajectory getHubTrajectory(){
//        return    drive.trajectoryBuilder(new Pose2d(new Vector2d(26, 66),Math.toRadians(180)))
//                .lineTo(new Vector2d(10, 65))
//                .splineToSplineHeading(new Pose2d(-10, 45, Math.toRadians(270)), Math.toRadians(270))
//                .build();
//    }
//
//    protected boolean toLineTeleop(double time){
//        return drive.toLineBlueTeleop(time);
//    }
//}
