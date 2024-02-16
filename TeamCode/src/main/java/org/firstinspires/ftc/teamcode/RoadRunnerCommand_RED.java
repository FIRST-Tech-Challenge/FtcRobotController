package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

public class RoadRunnerCommand_RED extends RoadRunnerSubsystem_RED {

    RoadRunnerCommand_RED(SampleMecanumDrive sampleDrive, HardwareMap hardwareMap, Pose2d HomePose, StartingPosition startingPosition, Path path, PixelStack pixelStack, ParkingPosition parkingPosition) {
        super(sampleDrive, hardwareMap, HomePose, startingPosition, path, pixelStack, parkingPosition);
    }

    public TrajectorySequenceBuilder spikeRandomizationPath(RoadRunnerSubsystem_RED.Randomization randomization){

        if (startingPosition == RoadRunnerSubsystem_RED.StartingPosition.SHORT){
            if (randomization == RoadRunnerSubsystem_RED.Randomization.LEFT){
                leftPixelSpike = leftPixel_SHORT;
                pixel_cycle_PoseTransfer = leftPixel_SHORT;
                leftSpikeStartingTangetValue = 0;
                leftSpikeFinalTangetValue = 0;
                return leftSpike;
            }
            else if (randomization == RoadRunnerSubsystem_RED.Randomization.CENTER){
                centerPixelSpike = centerPixel_SHORT;
                pixel_cycle_PoseTransfer = centerPixel_SHORT;
                return centerSpike;
            }
            else if (randomization == RoadRunnerSubsystem_RED.Randomization.RIGHT){
                rightPixelSpike = rightPixel_SHORT;
                pixel_cycle_PoseTransfer = rightPixel_SHORT;
                return rightSpike;
            }
        }
        else if (startingPosition == RoadRunnerSubsystem_RED.StartingPosition.LONG){
            if (randomization == RoadRunnerSubsystem_RED.Randomization.LEFT){
                rightPixelSpike = leftPixel_LONG;
                pixel_cycle_PoseTransfer = leftPixel_LONG;
                return rightSpike;
            }
            else if (randomization == RoadRunnerSubsystem_RED.Randomization.CENTER){
                centerPixelSpike = centerPixel_LONG;
                pixel_cycle_PoseTransfer = centerPixel_LONG;
                return centerSpike;
            }
            else if (randomization == RoadRunnerSubsystem_RED.Randomization.RIGHT){
                leftPixelSpike = rightPixel_LONG;
                pixel_cycle_PoseTransfer = rightPixel_LONG;
                leftSpikeStartingTangetValue = 1;
                leftSpikeFinalTangetValue = 1;
                return leftSpike;
            }
        }

        return leftSpike;
    }

    public TrajectorySequenceBuilder cycle(RoadRunnerSubsystem_RED.Randomization randomization){

        if (randomization == Randomization.LEFT){
            randomizedBackdrop = backdropLeft;
        }
        else if (randomization == Randomization.CENTER){
            randomizedBackdrop = backdropCenter;
        }
        else if (randomization == Randomization.RIGHT){
            randomizedBackdrop = backdropRight;
        }

        /*-----------------------------------------------------------*/

        if (path == Path.INNER){
            stationClose = stationClose_Inner;
            stationFar = stationFar_Inner;
            backdrop_Unload = backdropLeft;
            stackStation = stationInner;
            stackStationTangetValue = 0;
        }
        else if (path == Path.OUTER){
            stationClose = stationClose_Outer;
            stationFar = stationFar_Outer;
            backdrop_Unload = backdropRight;
            stackStation = stationOuter;
            stackStationTangetValue = 2;
        }

        /*-----------------------------------------------------------*/

        if (startingPosition == StartingPosition.SHORT){
            return pixel_backdrop_Short;
        }
        else if (startingPosition == StartingPosition.LONG){
            return pixel_backdrop_Long;
        }

        return pixel_backdrop_Short;
    }

    public TrajectorySequenceBuilder parking(){
        if (parkingPosition == RoadRunnerSubsystem_RED.ParkingPosition.INNER){
            parkingTangetValue = 0;
            parkingPose = parkingInner;
            return parking;
        }
        else if (parkingPosition == RoadRunnerSubsystem_RED.ParkingPosition.MID){
            parkingTangetValue = 1;
            parkingPose = parkingMiddle;
            return parking;
        }
        else if (parkingPosition == RoadRunnerSubsystem_RED.ParkingPosition.OUTER){
            parkingTangetValue = 1;
            parkingPose = parkingOuter;
            return parking;
        }

        return parking;
    }
}