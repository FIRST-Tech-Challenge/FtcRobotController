package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;

public class Main {
    public static void main(String[] args) {
        RCInLeft opMode = new RCInLeft();

        opMode.drive(new Pose2d());
    }
}
