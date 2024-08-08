package com.millburnx.pathplanner.components;

import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;

// Helper class to save the path
class PathSaver {
    public static void savePath(File file, ArrayList<BezierPoint> points) throws IOException {
        // Implement your path saving logic here
        // For example, you can write the points to a CSV file
        try (PrintWriter writer = new PrintWriter(file)) {
            for (BezierPoint bp : points) {
                writer.println(bp.point.x + "," + bp.point.y);
            }
        }
    }
}
