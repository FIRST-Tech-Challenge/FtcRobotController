#!/usr/bin/env python3
"""End-to-end desktop simulation of the Volley Consistency Test OpMode (~40 s, real time)."""
import sys
from pathlib import Path
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ftc_test_runner import run_tests

run_tests(('Launcher', 'Transfer', 'Intake', 'BatteryVoltage', 'BulkReads', 'FlywheelVelocityControl',
           'FlywheelIntegralControl', 'SynchronizedFeed', 'ShootingCoordinator', '../Tests/VolleyConsistencyTest'),
          Path(__file__).with_name('VolleyConsistencySim.java'), 'VolleyConsistencySim')
