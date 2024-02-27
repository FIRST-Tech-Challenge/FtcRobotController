package org.firstinspires.ftc.teamcode;

import android.util.Pair;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.ElevatorSubsystem;
import org.firstinspires.ftc.teamcode.CenterStageRobot.subsystems.OuttakeSusystem;
import org.firstinspires.ftc.teamcode.roadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.trajectorysequence.TrajectorySequenceBuilder;

public class RoadRunnerCommand_RED extends RoadRunnerSubsystem_RED {
    Telemetry telemetry;
    RoadRunnerCommand_RED(
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
                randomizedBackdrop = randomizationBackdropLeft;
                leftPixelSpike = leftPixel_SHORT;
                pixel_cycle_PoseTransfer = leftPixel_SHORT;
                leftSpikeStartingTangetValue = 0;
                leftSpikeFinalTangetValue = 0;
            }
            else if (randomization == Randomization.CENTER){
                randomizedBackdrop = randomizationBackdropCenter;
                centerPixelSpike = centerPixel_SHORT;
                pixel_cycle_PoseTransfer = centerPixel_SHORT;
            }
            else if (randomization == Randomization.RIGHT){
                randomizedBackdrop = randomizationBackdropRight;
                rightPixelSpike = rightPixel_SHORT;
                pixel_cycle_PoseTransfer = rightPixel_SHORT;
            }
        }
        else if (startingPosition == StartingPosition.LONG){
            if (randomization == Randomization.LEFT){
                randomizedBackdrop = randomizationBackdropLeft;
                rightPixelSpike = leftPixel_LONG;
                pixel_cycle_PoseTransfer = leftPixel_LONG;
            }
            else if (randomization == Randomization.CENTER){
                randomizedBackdrop = randomizationBackdropCenter;
                centerPixelSpike = centerPixel_LONG;
                pixel_cycle_PoseTransfer = centerPixel_LONG;
            }
            else if (randomization == Randomization.RIGHT){
                randomizedBackdrop = randomizationBackdropRight;
                leftPixelSpike = rightPixel_LONG;
                pixel_cycle_PoseTransfer = rightPixel_LONG;
                leftSpikeStartingTangetValue = 1;
                leftSpikeFinalTangetValue = 1;
            }
        }
    }

    public void cycle(){
        if (path == Path.INNER){
            stationClose = stationClose_Inner;
            stationFar = stationFar_Inner;
            backdrop_Unload = backdropLeft;
            stackStation = stationInner;
            stackStationSecondCycle = stationInnerSecondCycle;
            stackStationTangetValue = 0;
        }
        else if (path == Path.OUTER){
            stationClose = stationClose_Outer;
            stationFar = stationFar_Outer;
            backdrop_Unload = backdropRight;
            stackStation = stationOuter;
            stackStationSecondCycle = stationOuterSecondCycle;
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
        drive.followTrajectorySequenceAsync(getSpike(rand).build());
    }

    public void runSpike_RandomizedBackdrop() {
        drive.followTrajectorySequenceAsync(spike_randomizedBackdrop.build());
    }

    public void runBackdrop_Station_First_Cycle(){
        drive.followTrajectorySequenceAsync(backdrop_station_first_cycle.build());
    }

    public void runBackdrop_Station_Second_Cycle(){
        drive.followTrajectorySequenceAsync(backdrop_station_second_cycle.build());
    }

    public void runStation_Backdrop_First_Cycle(){
        drive.followTrajectorySequenceAsync(station_backdrop_first_cycle.build());
    }

    public void runStation_Backdrop_Second_Cycle(){
        drive.followTrajectorySequenceAsync(station_backdrop_second_cycle.build());
    }

    public void runParking(){
        drive.followTrajectorySequenceAsync(parking.build());
    }

    public void runSpike_Station(){
        drive.followTrajectorySequenceAsync(spike_station.build());
    }
}