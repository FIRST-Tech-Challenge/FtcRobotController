package com.millburnx.pathplanner;

import com.millburnx.pathplanner.components.PathEditor;

import javax.swing.*;

public class PathPlanner {
    public static void main(String[] args) {
        double ppi = 7.0;
        JFrame frame = new JFrame("Path Planner");

        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setSize(144 * (int) ppi, 144 * (int) ppi);
        frame.add(new PathEditor(ppi));
        frame.setVisible(true);
    }
}
