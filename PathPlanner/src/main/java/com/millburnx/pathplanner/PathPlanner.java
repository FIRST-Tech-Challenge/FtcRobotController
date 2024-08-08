package com.millburnx.pathplanner;

import com.millburnx.pathplanner.components.PathEditor;

import javax.swing.*;

public class PathPlanner extends JFrame {
    public PathPlanner() {
        setTitle("Path Planner");
        setSize(1000, 1000);
        setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        add(new PathEditor());
    }

    public void run() {
        SwingUtilities.invokeLater(() -> new PathPlanner().setVisible(true));
    }

    public static void main(String[] args) {
        PathPlanner pathPlanner = new PathPlanner();

        pathPlanner.run();
    }
}