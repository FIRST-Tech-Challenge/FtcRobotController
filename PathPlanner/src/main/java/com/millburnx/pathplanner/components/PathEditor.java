package com.millburnx.pathplanner.components;

import javax.imageio.ImageIO;
import javax.swing.*;
import javax.swing.border.Border;
import javax.swing.border.CompoundBorder;
import javax.swing.border.EmptyBorder;
import javax.swing.border.LineBorder;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.geom.Path2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Objects;

public class PathEditor extends JPanel {
    private ArrayList<BezierPoint> points = new ArrayList<>();
    private BezierPoint selectedPoint = null;
    private BufferedImage backgroundImage;

    public PathEditor() {
        try {
            backgroundImage = ImageIO.read(Objects.requireNonNull(getClass().getClassLoader().getResource("bg.png")));
        } catch (IOException e) {
            e.printStackTrace();
        }

        addMouseListener(new MouseAdapter() {
            public void mousePressed(MouseEvent e) {
                Point p = e.getPoint();
                for (BezierPoint bp : points) {
                    if (bp.point.distance(p) < 30) {
                        selectedPoint = bp;
                        return;
                    }
                }

                if (points.isEmpty()) {
                    points.add(new BezierPoint(p, false));
                } else {
                    points.add(new BezierPoint(new Point((int) (p.getX() - 50), (int) (p.getY() + 50)), true));
                    points.add(new BezierPoint(new Point((int) (p.getX() - 50), (int) (p.getY() - 50)), true));
                    points.add(new BezierPoint(p, false));
                }

                repaint();
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                selectedPoint = null;
            }
        });

        addMouseMotionListener(new MouseMotionAdapter() {
            public void mouseDragged(MouseEvent e) {
                if (selectedPoint != null) {
                    selectedPoint.point.setLocation(e.getPoint());
                    repaint();
                }
            }
        });

        JButton saveButton = getButton("SAVE PATH");
        saveButton.addActionListener(e -> savePath());
        add(saveButton, BorderLayout.PAGE_END);
    }

    private JButton getButton(String text) {
        JButton button = new JButton(text);
        button.setForeground(Color.BLACK);
        button.setBackground(Color.WHITE);
        button.setFont(new Font("Arial", Font.BOLD, 40));
        Border line = new LineBorder(Color.WHITE);
        Border margin = new EmptyBorder(5, 15, 5, 15);
        Border compound = new CompoundBorder(line, margin);
        button.setBorder(compound);
        return button;
    }

    private void savePath() {
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setDialogTitle("Save Path");
        int result = fileChooser.showSaveDialog(this);

        if (result == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile();
            try {
                // Save the path to the file
                PathSaver.savePath(file, points);
            } catch (IOException ex) {
                JOptionPane.showMessageDialog(this, "Error saving path: " + ex.getMessage());
            }
        }
    }

    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        if (backgroundImage != null) {
            g2d.drawImage(backgroundImage, 0, 0, getWidth(), getHeight(), this);
        }

        g2d.setStroke(new BasicStroke(5));

        g2d.setColor(Color.WHITE);
        if (points.size() >= 4) {
            Path2D path = new Path2D.Double();
            path.moveTo(points.get(0).point.x, points.get(0).point.y);
            for (int i = 1; i + 2 < points.size(); i += 3) {
                path.curveTo(
                        points.get(i).point.x, points.get(i).point.y,
                        points.get(i + 1).point.x, points.get(i + 1).point.y,
                        points.get(i + 2).point.x, points.get(i + 2).point.y
                );
            }
            g2d.draw(path);
        }

        for (BezierPoint bp : points) {
            if (bp.isControlPoint) {
                g2d.setColor(Color.MAGENTA);
            } else {
                g2d.setColor((Color.GREEN));
            }
            g2d.fillOval(bp.point.x - 5, bp.point.y - 5, 10, 10);
            g2d.drawOval(bp.point.x - 15, bp.point.y - 15, 30, 30);
        }
    }
}

