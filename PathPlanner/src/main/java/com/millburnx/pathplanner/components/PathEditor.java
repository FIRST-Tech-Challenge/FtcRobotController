package com.millburnx.pathplanner.components;

import com.millburnx.purePursuit.Utils.Point;

import javax.swing.*;
import javax.swing.border.EmptyBorder;
import java.awt.*;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class PathEditor extends JPanel {
    private final BezierPath bezierPath = new BezierPath();
    private BezierPoint selectedPoint = null;
    private boolean isPrevSelected = false;
    private boolean isNextSelected = false;

    private final List<List<BezierPoint>> undoStack = new ArrayList<>();

    public PathEditor() {
        setBackground(Color.DARK_GRAY);
        setLayout(new BorderLayout());

        addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                handleMousePressed(new Point(e.getPoint()));
            }

            @Override
            public void mouseReleased(MouseEvent e) {
                selectedPoint = null;
                isPrevSelected = false;
                isNextSelected = false;
            }
        });

        addMouseMotionListener(new MouseMotionAdapter() {
            @Override
            public void mouseDragged(MouseEvent e) {
                handleMouseDragged(new Point(e.getPoint()));
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
                isPrevSelected = false;
                isNextSelected = false;
                return;
            } else if (bp.isPrevNear(p)) {
                selectedPoint = bp;
                isPrevSelected = true;
                isNextSelected = false;
                return;
            } else if (bp.isNextNear(p)) {
                selectedPoint = bp;
                isPrevSelected = false;
                isNextSelected = true;
                return;
            }
        }

        // Adding a new point
        Double defaultHandleLength = 50.0;
        Point prevPoint = null;
        if (!bezierPath.getPoints().isEmpty()) {
            prevPoint = p.minus(new Point(defaultHandleLength, 0.0));
            BezierPoint lastPoint = bezierPath.getPoints().get(bezierPath.getPoints().size() - 1);
            lastPoint.setNextHandle(lastPoint.getAnchor().plus(new Point(defaultHandleLength, 0.0)));
        }
        BezierPoint newPoint = new BezierPoint(p, prevPoint, null);
        saveStateToUndoStack();
        bezierPath.addPoint(newPoint);

        repaint();
    }

    private void handleMouseDragged(Point p) {
        if (selectedPoint == null) {
            return;
        }
        if (isPrevSelected) {
            selectedPoint.setPreviousHandle(p);
        } else if (isNextSelected) {
            selectedPoint.setNextHandle(p);
        } else {
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
        FileDialog fileDialog = new FileDialog((Frame) null, "Save Path", FileDialog.SAVE);
        fileDialog.setDirectory("paths");
        fileDialog.setFile("*.tsv");
        fileDialog.setVisible(true);
        String filename = fileDialog.getFile();
        if (filename == null) {
            return;
        }

        File file = new File(fileDialog.getDirectory(), filename);
        try {
            // prev? anchor next ...
            List<Point> points = bezierPath.getPoints().stream()
                    .flatMap(bp -> Arrays.asList(
                            bp.getPreviousHandle(),
                            bp.getAnchor(),
                            bp.getNextHandle()
                    ).stream())
                    .filter(p -> p != null)
                    .collect(Collectors.toList());
            System.out.println("Saving to file: " + file.getAbsolutePath());
            System.out.println(points);
            Point.Companion.saveList(points, file);
        } catch (Exception ex) {
            JOptionPane.showMessageDialog(this, "Error saving path: " + ex.getMessage());

        }
    }

    private void loadPath() {
        FileDialog fileDialog = new FileDialog((Frame) null, "Load Path", FileDialog.LOAD);
        fileDialog.setDirectory("paths");
        fileDialog.setFile("*.tsv");
        fileDialog.setVisible(true);
        String filename = fileDialog.getFile();
        if (filename == null) {
            return;
        }

        File file = new File(fileDialog.getDirectory(), filename);

        try {
            saveStateToUndoStack();
            // TODO: Implement the path loading logic here
            List<Point> points = Point.Companion.loadList(file);
            List<BezierPoint> bezierPoints = new ArrayList<>();
            for (int i = 0; i < points.size(); i += 3) {
                Point prev = i == 0 ? null : points.get(i - 1);
                Point anchor = points.get(i);
                Point next = i + 1 < points.size() ? points.get(i + 1) : null;
                bezierPoints.add(new BezierPoint(anchor, prev, next));
            }
            bezierPath.getPoints().clear();
            bezierPath.getPoints().addAll(bezierPoints);
            repaint();
        } catch (Exception ex) {
            JOptionPane.showMessageDialog(this, "Error loading path: " + ex.getMessage());
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
