//package org.firstinspires.ftc.masters.oldAndUselessStuff;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//@Autonomous(name = "Red - Carousel 2 Warehouse (STATE)", group = "competition")
//public class RedCarousel2WarehouseOdo extends RedCarouselOdo {
//
//    protected void gotToPark(){
//
////        TrajectorySequence fromHubToWaitPos = drive.trajectorySequenceBuilder(drive.getLocalizer().getPoseEstimate())
////                .lineToSplineHeading(new Pose2d(new Vector2d(-24, -60), Math.toRadians(180)))
////                .build();
////        TrajectorySequence fromWaitPosToWarehouse = drive.trajectorySequenceBuilder(fromHubToWaitPos.end())
////                .lineToLinearHeading(new Pose2d( new Vector2d(48, -66), Math.toRadians(180)))
////                .build();
////
////        drive.followTrajectorySequence(fromHubToWaitPos);
////        double seconds = elapsedTime.seconds();
////        while (seconds<26) {
////            seconds = elapsedTime.seconds();
////        }
////        drive.followTrajectorySequence(fromWaitPosToWarehouse);
////        //drive.getCube(2000);
//    }
//
//
//}