package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

public class RoadRunnerCommand_BLUE extends RoadRunnerSubsystem_BLUE {
    Telemetry telemetry;
    RoadRunnerCommand_BLUE(
            SampleMecanumDrive sampleDrive, Pose2d HomePose,
            StartingPosition startingPosition, Path path, PixelStack pixelStack,
            ParkingPosition parkingPosition, Telemetry telemetry
    ) {
        super(sampleDrive, HomePose, startingPosition, path, pixelStack, parkingPosition);
        this.telemetry = telemetry;
    }

    public void spikeRandomizationPath(Randomization randomization){

        if (startingPosition == StartingPosition.SHORT){
            if (randomization == Randomization.LEFT){
                randomizedBackdrop = backdropLeft;
                leftPixelSpike = leftPixel_SHORT;
                pixel_cycle_PoseTransfer = leftPixel_SHORT;
                rightSpikeStartingTangetValue = 0;
                rightSpikeFinalTangetValue = 0;
            }
            else if (randomization == Randomization.CENTER){
                randomizedBackdrop = backdropCenter;
                centerPixelSpike = centerPixel_SHORT;
                pixel_cycle_PoseTransfer = centerPixel_SHORT;
            }
            else if (randomization == Randomization.RIGHT){
                randomizedBackdrop = backdropRight;
                rightPixelSpike = rightPixel_SHORT;
                pixel_cycle_PoseTransfer = rightPixel_SHORT;
            }
        }
        else if (startingPosition == StartingPosition.LONG){
            if (randomization == Randomization.LEFT){
                randomizedBackdrop = backdropLeft;
                backdrop_Unload = randomizedBackdrop;
                rightPixelSpike = leftPixel_LONG;
                pixel_cycle_PoseTransfer = leftPixel_LONG;
            }
            else if (randomization == Randomization.CENTER){
                randomizedBackdrop = backdropCenter;
                backdrop_Unload = randomizedBackdrop;
                centerPixelSpike = centerPixel_LONG;
                pixel_cycle_PoseTransfer = centerPixel_LONG;
            }
            else if (randomization == Randomization.RIGHT){
                randomizedBackdrop = backdropRight;
                backdrop_Unload = randomizedBackdrop;
                leftPixelSpike = rightPixel_LONG;
                pixel_cycle_PoseTransfer = rightPixel_LONG;
                rightSpikeStartingTangetValue = 1;
                rightSpikeFinalTangetValue = 1;
            }
        }
    }

    public void cycle(){

        if (path == Path.INNER){
            stationClose = stationClose_Inner;
            stationFar = stationFar_Inner;
            backdrop_Unload = backdropRight;
            stackStation = stationInner;
            stackStationTangetValue = 0;
        }
        else if (path == Path.OUTER){
            stationClose = stationClose_Outer;
            stationFar = stationFar_Outer;
            backdrop_Unload = backdropLeft;
            stackStation = stationOuter;
            stackStationTangetValue = 2;
        }
    }

    public void parking(){
        if (parkingPosition == ParkingPosition.INNER){
            parkingTangetValue = 0;
            parkingPose = parkingInner;
        }
        else if (parkingPosition == ParkingPosition.MID){
            parkingTangetValue = 1;
            parkingPose = parkingMiddle;
        }
        else if (parkingPosition == ParkingPosition.OUTER){
            parkingTangetValue = 1;
            parkingPose = parkingOuter;
        }
    }

    public TrajectorySequenceBuilder getSpike(Randomization randomization){
        if (startingPosition == StartingPosition.SHORT){
            if (randomization == Randomization.LEFT){
                return leftSpike;
            }
            else if (randomization == Randomization.CENTER){
                return centerSpike;
            }
            else if (randomization == Randomization.RIGHT){
                return rightSpike;
            }
        }
        else if (startingPosition == StartingPosition.LONG){
            if (randomization == Randomization.LEFT){
                return rightSpike;
            }
            else if (randomization == Randomization.CENTER){
                return centerSpike;
            }
            else if (randomization == Randomization.RIGHT){
                return leftSpike;
            }
        }

        return leftSpike;
    }

    public void runSpike(Randomization rand) {
        drive.followTrajectorySequence(getSpike(rand).build());
    }

    public void runSpike_RandomizedBackdrop() {
        drive.followTrajectorySequence(spike_randomizedBackdrop.build());
    }

    public void runBackdrop_Station(){
        drive.followTrajectorySequence(backdrop_station.build());
    }

    public void runStation_Backdrop(){
        drive.followTrajectorySequence(station_backdrop.build());
    }

    public void runParking(){
        drive.followTrajectorySequence(parking.build());
    }

    public void runSpike_Station(){
        drive.followTrajectorySequence(spike_station.build());
    }
}