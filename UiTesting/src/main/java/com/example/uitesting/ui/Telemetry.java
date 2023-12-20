package com.example.uitesting.ui;

import java.awt.Canvas;
import java.awt.Font;
import java.awt.Graphics;
import java.util.ArrayList;

/**
 * This class implements a lightweight emulation of FTC Telemetry that can run on the PC.
 */
public class Telemetry {
    WindowFrame windowFrame;
    Canvas canvas;
    ArrayList<String> lineList = new ArrayList<>();

    public Telemetry() {
        windowFrame = new WindowFrame("UI", 800);
        windowFrame.setVisible(true);
    }

    public void addLine(String string) {
        lineList.add(string);
    }

    public void update() {
        int FONT_SIZE = 14;
        canvas = windowFrame.getCanvas();
        Graphics g = canvas.getBufferStrategy().getDrawGraphics();
        g.clearRect(0, 0, canvas.getWidth(), canvas.getHeight());
        g.setFont(new Font("Sans", Font.BOLD, FONT_SIZE));

        int x = 100;
        int y = 100;
        for (String line : lineList) {
            g.drawString(line, x, y);
            y += FONT_SIZE;
        }
        g.dispose();
        canvas.getBufferStrategy().show();
        lineList.clear();
    }
}
