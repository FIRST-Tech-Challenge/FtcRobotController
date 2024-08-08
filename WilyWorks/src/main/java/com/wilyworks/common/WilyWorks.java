package com.wilyworks.common;

import static java.lang.ClassLoader.getSystemClassLoader;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2dDual;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;

/**
 * FTC programs use this class to configure and interface with the Wily Works simulator.
 */
public class WilyWorks {
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Configuration

    /**
     * Derive from this class and mark it with the @Wily annotation to configure the Wily Works
     * simulator.
     */
    public static class Config {
        // Set these to the actual dimensions of your robot:
        public double robotWidth = 18.0;
        public double robotLength = 18.0;

        // Maximum linear and rotational speeds in inches/s and radians/s, respectively:
        public double maxLinearSpeed = 60;
        public double maxAngularSpeed = Math.PI;

        // Maximum linear acceleration and deceleration, in inches/s/s:
        public double maxLinearAcceleration = 50;
        public double maxLinearDeceleration = -30; // Should be negative

        // Maximum angular acceleration and deceleration, in radians/s/s:
        public double maxAngularAcceleration = Math.PI;

        // Fill this out to describe cameras on the robot:
        public Camera[] cameras = {
            // new Camera("CameraExample", 3, 4, 0, Math.toRadians(120), 0.120)
        };

        public DistanceSensor[] distanceSensors = {
            // new DistanceSensor("DistanceExample", 1, 2, Math.toRadians(90));
        };

        /**
         * Structure used to describe April Tag cameras on the robot:
         */
        static public class Camera {
            // Camera name as specified in the robot's configuration:
            public String name;

            // Camera position in inches relative to the robot's center of rotation.
            // Positive 'x' is towards the front of the robot, negative towards the back.
            // Positive 'y' is towards the left of the robot, negative towards the right:
            public double x;
            public double y;

            // Orientation of the camera relative to the front of the robot, in radians. If zero,
            // the camera points straight forward; if Pi, the camera points straight backwards:
            public double orientation;

            // Field of view of the camera, in radians. Can be zero which assigns a default:
            public double fieldOfView;

            // Latency of the April Tag processing for this camera, in seconds:
            public double latency;

            public Camera(String name, double x, double y, double orientation, double fieldOfView, double latency) {
                this.name = name; this.x = x; this.y = y; this.orientation = orientation; this.fieldOfView = fieldOfView; this.latency = latency;
            }
        }

        /**
         * Structure used to describe distance sensors on the robot:
         */
        static public class DistanceSensor {
            // Distance sensor name as specified in the robot's configuration:
            public String name;

            // Sensor position in inches relative to the robot's center of rotation.
            // Positive 'x' is towards the front of the robot, negative towards the back.
            // Positive 'y' is towards the left of the robot, negative towards the right:
            public double x;
            public double y;

            // Orientation of the sensor relative to the front of the robot, in radians. If zero,
            // the sensor points straight forward; if Pi, the sensor points straight backwards:
            public double orientation;

            public DistanceSensor(String name, double x, double y, double orientation) {
                this.name = name; this.x = x; this.y = y; this.orientation = orientation;
            }
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Interaction

    // WilyLink class for communicating with the Wily Works simulator:
    static private Class<?> wilyCore = getWilyCore();

    // Check this boolean to determine whether you're running on the real robot or in a simulation:
    static public boolean isSimulating = (wilyCore != null);

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Implementation

    // Wrap WilyLink initialization with
    static Class<?> getWilyCore() {
        try {
            return getSystemClassLoader().loadClass("com.wilyworks.simulator.WilyCore");
        } catch (ClassNotFoundException e) {
            return null;
        }
    }

    // The pose is always field-relative:
    static public class Pose {
        double x, y; // Inches
        double heading; // Radians

        public Pose(double x, double y, double heading) {
            this.x = x; this.y = y; this.heading = heading;
        }
    }

    // The velocity will either be robot-relative or field-relative, depending on the API used:
    static public class Velocity {
        double x, y; // Inches/s
        double angular; // Radians/s
        public Velocity(double x, double y, double angular) {
            this.x = x; this.y = y; this.angular = angular;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Control

    // Set the robot to a given pose and (optional) velocity in the simulation. The
    // localizer will not register a move.
    @SuppressWarnings("UnusedReturnValue")
    static public boolean setStartPose(Pose2d pose, PoseVelocity2d velocity) {
        if (wilyCore != null) {
            try {
                Method setPose = wilyCore.getMethod("setStartPose",
                        double.class, double.class, double.class,
                        double.class, double.class, double.class);
                setPose.invoke(null,
                        pose.position.x, pose.position.y, pose.heading.log(),
                        velocity.linearVel.x, velocity.linearVel.y, velocity.angVel);
                return true; // ====>
            } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return false;
    }

    // MecanumDrive uses this while running a trajectory to update the simulator as to its
    // current intermediate pose and velocity. This update will be reflected in the localizer
    // results.
    @SuppressWarnings("UnusedReturnValue")
    static public boolean runTo(Pose2d pose, PoseVelocity2d velocity) {
        if (wilyCore != null) {
            try {
                Method setPose = wilyCore.getMethod("runTo",
                        double.class, double.class, double.class,
                        double.class, double.class, double.class);
                setPose.invoke(null,
                        pose.position.x, pose.position.y, pose.heading.log(),
                        velocity.linearVel.x, velocity.linearVel.y, velocity.angVel);
                return true; // ====>
            } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return false;
    }

    // Get the simulation's true pose:
    static public Pose2d getPose() {
        if (wilyCore != null) {
            try {
                Method getPose = wilyCore.getMethod("getPose");
                return (Pose2d) getPose.invoke(null);
            } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return null;
    }

    // Get the simulation's true pose:
    static public PoseVelocity2d getPoseVelocity() {
        if (wilyCore != null) {
            try {
                Method getPose = wilyCore.getMethod("getPoseVelocity");
                return (PoseVelocity2d) getPose.invoke(null);
            } catch (IllegalAccessException | InvocationTargetException | NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return null;
    }

    // Set the drive powers:
    static public boolean setDrivePowers(
            // Manual power, normalized voltage from -1 to 1, robot-relative coordinates, can be null:
            PoseVelocity2d stickVelocity,
            // Computed power, inches/s and radians/s, field-relative coordinates, can be null:
            PoseVelocity2d assistVelocity) {
        if (wilyCore != null) {
            try {
                Method setDrivePowers = wilyCore.getMethod("setDrivePowers",
                        PoseVelocity2d.class, PoseVelocity2d.class);
                setDrivePowers.invoke(null, stickVelocity, assistVelocity);
                return true; // ====>
            } catch (InvocationTargetException|IllegalAccessException|NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return false;
    }

    // Get the localizer position and velocity from the simulation:
    static public Twist2dDual<Time> localizerUpdate() {
        if (wilyCore != null) {
            try {
                Method getLocalization = wilyCore.getMethod("getLocalization");
                double[] localization = (double[]) getLocalization.invoke(null);
                return new Twist2dDual<>(new Vector2dDual<>(
                        new DualNum<Time>(new double[] { localization[0], localization[3] }),
                        new DualNum<Time>(new double[] { localization[1], localization[4] })),
                        new DualNum<Time>(new double[] { localization[2], localization[5] }));

            } catch (InvocationTargetException|IllegalAccessException|NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
        return null;
    }

    // Ask the simulation to update by a specified amount of time:
    static public void updateSimulation(double deltaTime) {
        if (wilyCore != null) {
            try {
                Method updateSimulation = wilyCore.getMethod("updateSimulation", double.class);
                updateSimulation.invoke(null, deltaTime);
            } catch (InvocationTargetException|IllegalAccessException|NoSuchMethodException e) {
                throw new RuntimeException(e);
            }
        }
    }
}
