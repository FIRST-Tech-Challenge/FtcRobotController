package com.millburnx.pathplanner;

import com.millburnx.pathplanner.components.PathEditor;
import com.millburnx.purePursuit.PurePursuit;

import javax.swing.*;

public class PathPlanner {
    public static void main(String[] args) {
        SwingUtilities.invokeLater(() -> {
            JFrame frame = new JFrame("Path Planner");
            double ppi = 5.0;
            PurePursuit purePursuit = new PurePursuit(ppi, -1.0);

            frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame.setSize(800, 600);
            frame.add(new PathEditor());
            frame.setVisible(true);

            JFrame frame2 = new JFrame("Pure Pursuit");
            frame2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame2.setSize((int) (144 * ppi), (int) (144 * ppi));
            frame2.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            frame2.setLocationRelativeTo(null);
            frame2.getContentPane().add(purePursuit.getFtcDashboard().getPanel());
            frame2.setVisible(true);

            // I have no idea why this needs to be multithreaded, but it just breaks rendering for both windows if it isn't
            Thread one = new Thread(purePursuit::start);
            one.start();
        });
    }
}
