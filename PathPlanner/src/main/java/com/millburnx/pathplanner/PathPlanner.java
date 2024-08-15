package com.millburnx.pathplanner;

import com.millburnx.pathplanner.components.PathEditor;

import javax.swing.*;

public class PathPlanner {
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Path Planner");
            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setSize(800, 600);
            frame.add(new PathEditor());
            frame.setVisible(true);
        });
    }
}
