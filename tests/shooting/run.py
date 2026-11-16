#!/usr/bin/env python3
"""Competition shooting regression with real FTC SDK interfaces and simulated motors."""
import sys
from pathlib import Path
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ftc_test_runner import run_tests

run_tests(('Launcher', 'Transfer', 'Intake', 'BatteryVoltage', 'FlywheelVelocityControl',
           'FlywheelIntegralControl', 'SynchronizedFeed', 'ShootingCoordinator'),
          Path(__file__).with_name('CompetitionShootingRegression.java'), 'CompetitionShootingRegression')

run_tests(('SynchronizedFeed', 'BatteryVoltage'),
          Path(__file__).with_name('PacedFeedRegression.java'), 'PacedFeedRegression')
