#!/usr/bin/env python3
"""
Test: Manual axis jog dialog behavior after fix
"""
import sys
import numpy as np
from pathlib import Path

# Ensure workspace is in path
workspace = Path(__file__).parent
sys.path.insert(0, str(workspace))

from PySide6.QtWidgets import QApplication
from gui.dialog.axisconfigdialog import AxisConfigDialog


def test_todos_50_percent():
    """Test that 'Todos 50%' button sets 50% correctly"""
    app = QApplication.instance() or QApplication(sys.argv)
    dialog = AxisConfigDialog()
    
    print("\n=== Test: Todos 50% Button ===")
    
    # Enable axes 0, 1, 2
    for i in range(3):
        dialog.checks[i].setChecked(True)
    
    # Call home_enabled (Todos 50%)
    dialog.home_enabled()
    
    # Verify
    for i in range(6):
        expected = 50 if i < 3 else 0  # First 3 enabled, rest at 0
        actual = dialog.spins[i].value()
        status = "[PASS]" if actual == expected else "[FAIL]"
        print("  {} Eje {}: {}% (expected {}%)".format(status, i+1, actual, expected))
        assert actual == expected, "Eje {} should be {}%, got {}%".format(i+1, expected, actual)
    
    print("  [OK] Todos 50% works correctly")


def test_axis_values_preserved():
    """Test that disabled axes preserve their last value"""
    app = QApplication.instance() or QApplication(sys.argv)
    dialog = AxisConfigDialog()
    
    print("\n=== Test: Disabled Axes Preserve Values ===")
    
    # Set custom values
    for i in range(6):
        dialog.spins[i].setValue(10 * (i + 1))  # 10, 20, 30, 40, 50, 60
    
    # Enable only axes 0 and 2
    dialog.checks[0].setChecked(True)
    dialog.checks[2].setChecked(True)
    
    # Call home_enabled (should only set enabled axes to 50)
    dialog.home_enabled()
    
    # Verify
    expected = [50, 20, 50, 40, 50, 60]  # Axes 0 and 2 set to 50, others keep values
    vector = dialog.get_command_vector()
    
    for i in range(6):
        status = "[PASS]" if vector[i] == expected[i] else "[FAIL]"
        print("  {} Eje {}: {}% (expected {}%)".format(status, i+1, vector[i], expected[i]))
        assert vector[i] == expected[i], "Eje {} should be {}%, got {}%".format(i+1, expected[i], vector[i])
    
    print("  [OK] Disabled axes preserve their values")


def test_single_axis_jog():
    """Test: Enable only Axis 1, set to 10%, verify command vector"""
    app = QApplication.instance() or QApplication(sys.argv)
    dialog = AxisConfigDialog()
    
    print("\n=== Test: Single Axis Jog (Eje 1 = 10%) ===")
    
    # Initial state: all at 0 (HOME)
    initial_vector = dialog.get_command_vector()
    print("  Initial vector: {}".format(initial_vector))
    
    # Enable only Eje 1
    dialog.checks[0].setChecked(True)
    
    # Set Eje 1 to 10%
    dialog.spins[0].setValue(10)
    
    # Get command vector
    vector = dialog.get_command_vector()
    expected = [10, 0, 0, 0, 0, 0]
    
    print("  After: Eje 1 enabled, value=10%")
    print("  Command vector: {}".format(vector))
    print("  Expected:       {}".format(expected))
    
    # Verify
    for i in range(6):
        status = "[PASS]" if vector[i] == expected[i] else "[FAIL]"
        print("  {} Eje {}: {}%".format(status, i+1, vector[i]))
        assert vector[i] == expected[i], "Eje {} should be {}%, got {}%".format(i+1, expected[i], vector[i])
    
    print("  [OK] Single axis jog works correctly")


def test_axes_changed_signal():
    """Test that axes_changed signal is emitted"""
    app = QApplication.instance() or QApplication(sys.argv)
    dialog = AxisConfigDialog()
    
    print("\n=== Test: axes_changed Signal ===")
    
    # Create a flag to track signal emission
    signal_emitted = False
    
    def on_axes_changed():
        nonlocal signal_emitted
        signal_emitted = True
    
    # Connect to signal
    dialog.axes_changed.connect(on_axes_changed)
    
    # Trigger signal by changing spinbox
    dialog.spins[0].setValue(25)
    
    # Verify
    status = "[PASS]" if signal_emitted else "[FAIL]"
    print("  {} Signal emitted on spinbox change: {}".format(status, signal_emitted))
    assert signal_emitted, "axes_changed signal should be emitted"
    
    print("  [OK] axes_changed signal works correctly")


if __name__ == "__main__":
    try:
        print("=" * 60)
        print("MANUAL AXIS JOG - REGRESSION TEST")
        print("=" * 60)
        
        test_todos_50_percent()
        test_axis_values_preserved()
        test_single_axis_jog()
        test_axes_changed_signal()
        
        print("\n" + "=" * 60)
        print("ALL TESTS PASSED [OK]")
        print("=" * 60)
    except Exception as e:
        print("\n[FAIL] TEST FAILED: {}".format(e))
        import traceback
        traceback.print_exc()
        sys.exit(1)
