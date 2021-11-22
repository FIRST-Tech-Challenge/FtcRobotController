package org.firstinspires.ftc.teamcode.drive;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.roadrunner.drive.MecanumDrive;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionSegment;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.mechanism.MB1242;

import java.util.ArrayList;
import java.util.List;

public class UltrasonicLocalizer implements Localizer {
    ElapsedTime pingTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    MB1242 sensor1;
    MB1242 sensor2;
    MB1242 sensor3;
    MB1242 sensor4;

    MecanumDrive drive;

    /**
     *
     * @param sensors An array of 4 MB1242 ultrasonic sensors, in order of the robot's front, right, back, left
     * @param drive The MecanumDrive object to use for localization
     */
    UltrasonicLocalizer(MB1242[] sensors, MecanumDrive drive) {
        sensor1 = sensors[0];
        sensor2 = sensors[1];
        sensor3 = sensors[2];
        sensor4 = sensors[3];
        this.drive = drive;
    }

    double x = 0;
    double y = 0;
    double theta = 0;
    Pose2d previousPose = new Pose2d(0, 0, 0);
    Pose2d poseEstimate = new Pose2d(0, 0, 0);
    List<Double> lastWheelPositions = new ArrayList<>();


    public void init(HardwareMap hardwareMap) {
        pingTimer.reset();
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    // A guy on discord said this is about right
    int MILLIS_PER_CYCLE = 34;

    void pingSensors() {
        sensor1.ping();
        sensor2.ping();
        sensor3.ping();
        sensor4.ping();
    }

    final double ROBOT_WIDTH = 11.95;
    final double ROBOT_LENGTH = 13.8;

    // This gets the distances from all 4 sensors and adds the robot width and length
    // to the distances where necessary
    double[] getSensorDistances() {
        double[] distances = new double[4];
        distances[0] = sensor1.getDistance(DistanceUnit.INCH);
        distances[1] = sensor2.getDistance(DistanceUnit.INCH);
        distances[2] = sensor3.getDistance(DistanceUnit.INCH);
        distances[3] = sensor4.getDistance(DistanceUnit.INCH);
        distances[0] += ROBOT_LENGTH / 2;
        distances[1] += ROBOT_WIDTH / 2;
        distances[2] += ROBOT_LENGTH / 2;
        distances[3] += ROBOT_WIDTH / 2;
        return distances;
    }


    @NonNull
    @Override
    public Pose2d getPoseEstimate() {
        return poseEstimate;
    }

    @Override
    public void setPoseEstimate(@NonNull Pose2d pose2d) {
        previousPose = pose2d;
        poseEstimate = pose2d;
        lastWheelPositions = new ArrayList<>();
    }

    @Nullable
    @Override
    public Pose2d getPoseVelocity() {
        return null;
    }

    @Override
    public void update() {
        // Calculate the position of the robot based on the ultrasonic sensors and the IMU values
        // Remember that the field is 12 x 12 feet, and the robot is 11.95 x 13.8 inches
        // The minimum distance detectable by the ultrasonic sensors is 7.874 inches
        // The maximum distance is irrelevant, as the robot will never be that far away from a wall
        theta = drive.getExternalHeading();
        if (pingTimer.time() > MILLIS_PER_CYCLE) {
            pingTimer.reset();
            pingSensors();
            double[] distances = getSensorDistances();
            ArrayList<Pose2d> averagablePoses = new ArrayList<>();
            // All the different pairs of distances we can use to calculate the position of the robot
            double[] set1 = {distances[0], distances[1]};
            double[] set2 = {distances[1], distances[2]};
            double[] set3 = {distances[2], distances[3]};
            double[] set4 = {distances[3], distances[0]};
            double[] set5 = {distances[0], distances[2]};
            double[] set6 = {distances[1], distances[3]};
            // Make a list of them
            double[][] sets = {set1, set2, set3, set4, set5, set6};
            // For each set, calculate the position of the robot that you could get from those
            for (double[] set : sets) {
                double x1 = set[0] * Math.cos(theta);
                double y1 = set[0] * Math.sin(theta);
                double x2 = set[1] * Math.cos(theta);
                double y2 = set[1] * Math.sin(theta);
                // If the x or y values are very similar, the two sensors are probably facing the same wall
                // And the pose they calculate is probably wrong, so we don't use it
                if (Math.abs(x1 - x2) < 3) {
                    continue;
                }
                if (Math.abs(y1 - y2) < 3) {
                    continue;
                }
                // We assume that the larger distance is the one that is more likely to be the wall
                // Rather than an object in the way
                double realX = Math.abs(x1) > Math.abs(x2) ? x1 : x2;
                double realY = Math.abs(y1) > Math.abs(y2) ? y1 : y2;
                // If the distance is negative, it means that the robot is that distance from the opposite wall
                // So we add 144 (the length/width of the field) to the distance to get the real distance
                if (realX > 0) {
                    realX = 144 + realX;
                }
                if (realY > 0) {
                    realY = 144 + realY;
                }

                Pose2d newPose = new Pose2d(realX, realY, theta);
                double poseDifference = newPose.vec().distTo(previousPose.vec());
                // If the pose difference is too big, we don't use it, since again, it's probably wrong
                if (poseDifference < 1) {
                    averagablePoses.add(newPose);
                }
            }
            // At the end, we average all the poses we found
            if (averagablePoses.size() > 0) {
                double xAverage = 0;
                double yAverage = 0;
                double thetaAverage = 0;
                for (Pose2d pose : averagablePoses) {
                    xAverage += pose.getX();
                    yAverage += pose.getY();
                    thetaAverage += pose.getHeading();
                }
                xAverage /= averagablePoses.size();
                yAverage /= averagablePoses.size();
                thetaAverage /= averagablePoses.size();
                poseEstimate = new Pose2d(xAverage, yAverage, thetaAverage);
            } else {
                // If we didn't find any poses, we use the enocoder values instead
                poseEstimate = calculatePoseEncoders();
            }
        } else {
            // If we haven't waited long enough, we use the encoder values instead
            poseEstimate = calculatePoseEncoders();
        }
        previousPose = poseEstimate;
    }
    public Pose2d calculatePoseEncoders() {
        // If we haven't waited long enough, we calculate the pose based on the drive encoders
        List<Double> wheelPositions = drive.getWheelPositions();
        List<Double> wheelDeltas = new ArrayList<>();
        if (lastWheelPositions.size() != 0) {
            for (int i = 0; i < lastWheelPositions.size(); i++) {
                double difference = wheelPositions.get(i) - lastWheelPositions.get(i);
                wheelDeltas.add(difference);
            }
        }
        Pose2d robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                wheelDeltas,
                DriveConstants.TRACK_WIDTH,
                DriveConstants.TRACK_WIDTH
        );
        double finalHeadingDelta = Angle.normDelta(robotPoseDelta.getHeading() - previousPose.getHeading());
        lastWheelPositions = wheelPositions;
        return Kinematics.relativeOdometryUpdate(poseEstimate, new Pose2d(robotPoseDelta.vec(), finalHeadingDelta));
    }
}
