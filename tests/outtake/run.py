#!/usr/bin/env python3
"""Test the small production outtake addition without modifying the experimental sandbox."""
import sys
sys.dont_write_bytecode = True
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ftc_test_runner import run_tests

run_tests(('Launcher', 'Transfer', 'Intake', 'BatteryVoltage', 'FlywheelVelocityControl', 'FlywheelIntegralControl'),
          Path(__file__).with_name('OuttakeRegression.java'), 'OuttakeRegression')
