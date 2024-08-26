package com.acmerobotics.dashboard;

public class TestServer {
    public static void main(String[] args) throws InterruptedException {
        TestDashboardInstance.getInstance().start();
    }
}
