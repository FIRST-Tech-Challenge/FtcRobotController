package com.millburnx.pathplanner.components;

import javax.swing.*;
import javax.swing.border.EmptyBorder;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class PathEditor extends JPanel {
    private final BezierPath bezierPath = new BezierPath();
    private BezierPoint selectedPoint = null;
    private Point selectedHandle = null;
    private boolean isHandle1Selected = false;

    private final List<List<BezierPoint>> undoStack = new ArrayList<>();

    public PathEditor() {
        setBackground(Color.DARK_GRAY);
        setLayout(new BorderLayout());

        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                handleMousePressed(e.getPoint());
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                selectedPoint = null;
                selectedHandle = null;
            }
        });

        addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseDragged(MouseEvent e) {
                handleMouseDragged(e.getPoint());
            }
        });

        JPanel buttonPanel = new JPanel();
        buttonPanel.setLayout(new FlowLayout(FlowLayout.LEFT));

        JButton loadButton = createModernButton("Load Path");
        loadButton.addActionListener(e -> loadPath());
        buttonPanel.add(loadButton);

        JButton saveButton = createModernButton("Save Path");
        saveButton.addActionListener(e -> savePath());
        buttonPanel.add(saveButton);

        JButton undoButton = createModernButton("Undo");
        undoButton.addActionListener(e -> undo());
        buttonPanel.add(undoButton);

        add(buttonPanel, BorderLayout.NORTH);
    }

    private void handleMousePressed(Point p) {
        for (BezierPoint bp : bezierPath.getPoints()) {
            if (bp.isAnchorNear(p)) {
                selectedPoint = bp;
                return;
            } else if (bp.isHandle1Near(p)) {
                selectedHandle = bp.getHandle1();
                isHandle1Selected = true;
                return;
            } else if (bp.isHandle2Near(p)) {
                selectedHandle = bp.getHandle2();
                isHandle1Selected = false;
                return;
            }
        }

        // Adding a new point
        Point handle1 = new Point(p.x + 50, p.y);
        Point handle2 = null;

        int pointCount = bezierPath.getPoints().size();
        if (pointCount == 0) {
            handle2 = null;
        } else {
            handle2 = new Point(p.x - 50, p.y);
        }

        BezierPoint newPoint = new BezierPoint(p, handle1, handle2);
        saveStateToUndoStack();
        bezierPath.addPoint(newPoint);

        if (pointCount > 0) {
            BezierPoint previousPoint = bezierPath.getPoints().get(pointCount - 1);
            if (previousPoint.getHandle2() == null && pointCount > 1) {
                previousPoint.setHandle2(new Point(previousPoint.getAnchor().x - 50, previousPoint.getAnchor().y));
            }
        }

        if (pointCount > 0) {
            BezierPoint lastPoint = bezierPath.getPoints().get(bezierPath.getPoints().size() - 1);
            if (lastPoint.getHandle2() == null) {
                lastPoint.setHandle1(new Point(lastPoint.getAnchor().x + 50, lastPoint.getAnchor().y));
            }
        }

        repaint();
    }

    private void handleMouseDragged(Point p) {
        if (selectedHandle != null) {
            selectedHandle.setLocation(p);
        } else if (selectedPoint != null) {
            selectedPoint.setAnchor(p);
        }
        repaint();
    }

    private JButton createModernButton(String text) {
        JButton button = new JButton(text);
        button.setForeground(Color.WHITE);
        button.setBackground(new Color(33, 150, 243));
        button.setFont(new Font("Arial", Font.BOLD, 16));
        button.setFocusPainted(false);
        button.setBorder(new EmptyBorder(10, 20, 10, 20));
        button.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        button.setContentAreaFilled(false);
        button.setOpaque(true);

        button.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseEntered(MouseEvent e) {
                button.setBackground(new Color(30, 136, 229));
            }

            @Override
            public void mouseExited(MouseEvent e) {
                button.setBackground(new Color(33, 150, 243));
            }
        });

        return button;
    }

    private void savePath() {
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setDialogTitle("Save Path");
        int result = fileChooser.showSaveDialog(this);

        if (result == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile();
            try {
                // Implement the path saving logic here
            } catch (Exception ex) {
                JOptionPane.showMessageDialog(this, "Error saving path: " + ex.getMessage());
            }
        }
    }

    private void loadPath() {
        JFileChooser fileChooser = new JFileChooser();
        fileChooser.setDialogTitle("Load Path");
        int result = fileChooser.showOpenDialog(this);

        if (result == JFileChooser.APPROVE_OPTION) {
            File file = fileChooser.getSelectedFile();
            try {
                saveStateToUndoStack();
                // Implement the path loading logic here
                // bezierPath.loadPath(file);
                repaint();
            } catch (Exception ex) {
                JOptionPane.showMessageDialog(this, "Error loading path: " + ex.getMessage());
            }
        }
    }

    private void undo() {
        if (!undoStack.isEmpty()) {
            List<BezierPoint> previousPoints = undoStack.remove(undoStack.size() - 1);
            bezierPath.getPoints().clear();
            bezierPath.getPoints().addAll(previousPoints);
            repaint();
        }
    }

    private void saveStateToUndoStack() {
        List<BezierPoint> pointsSnapshot = new ArrayList<>();
        for (BezierPoint bp : bezierPath.getPoints()) {
            pointsSnapshot.add(bp.clone()); // Ensure BezierPoint has a proper clone method
        }
        undoStack.add(pointsSnapshot);
    }

    @Override
    protected void paintComponent(Graphics g) {
        super.paintComponent(g);
        Graphics2D g2d = (Graphics2D) g;
        g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

        bezierPath.draw(g2d);
        bezierPath.drawHandles(g2d);
    }
}
