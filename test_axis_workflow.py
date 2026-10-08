#!/usr/bin/env python3
"""
Test: Full axis jog workflow with MainWindow logic
"""
import sys
from pathlib import Path

workspace = Path(__file__).parent
sys.path.insert(0, str(workspace))

from PySide6.QtWidgets import QApplication
from gui.dialog.axisconfigdialog import AxisConfigDialog


def test_axis_jog_workflow():
    """Test complete axis jog workflow"""
    print("\n=== Test: Complete Axis Jog Workflow ===")
    
    app = QApplication.instance() or QApplication(sys.argv)
    dialog = AxisConfigDialog()
    
    # Initial state: all axes at 0%
    print("\n1. Initial state:")
    vector = dialog.get_command_vector()
    print("   Command vector: {}".format(vector))
    assert all(v == 0 for v in vector), "All axes should start at 0%"
    
    # Step 1: Enable Eje 1 and set to 10%
    print("\n2. Enable Eje 1, set to 10%:")
    dialog.checks[0].setChecked(True)
    dialog.spins[0].setValue(10)
    vector = dialog.get_command_vector()
    print("   Enabled axes: {}".format(dialog.get_enabled()))
    print("   Command vector: {}".format(vector))
    assert vector == [10, 0, 0, 0, 0, 0], "Should be [10, 0, 0, 0, 0, 0]"
    
    # Step 2: Enable Eje 2 and set to 20%
    print("\n3. Enable Eje 2, set to 20%:")
    dialog.checks[1].setChecked(True)
    dialog.spins[1].setValue(20)
    vector = dialog.get_command_vector()
    print("   Enabled axes: {}".format(dialog.get_enabled()))
    print("   Command vector: {}".format(vector))
    assert vector == [10, 20, 0, 0, 0, 0], "Should be [10, 20, 0, 0, 0, 0]"
    
    # Step 3: Disable Eje 1 (value should persist)
    print("\n4. Disable Eje 1 (value persists):")
    dialog.checks[0].setChecked(False)
    vector = dialog.get_command_vector()
    print("   Enabled axes: {}".format(dialog.get_enabled()))
    print("   Command vector: {}".format(vector))
    assert vector == [10, 20, 0, 0, 0, 0], "Disabled axis should keep value [10, 20, 0, 0, 0, 0]"
    
    # Step 4: Use "Todos 50%" with Eje 2 enabled
    print("\n5. Click 'Todos 50%' with only Eje 2 enabled:")
    dialog.checks[1].setChecked(True)
    dialog.home_enabled()
    vector = dialog.get_command_vector()
    print("   Enabled axes: {}".format(dialog.get_enabled()))
    print("   Command vector: {}".format(vector))
    # Eje 1 disabled (persists at 10), Eje 2 enabled (set to 50)
    assert vector == [10, 50, 0, 0, 0, 0], "Should be [10, 50, 0, 0, 0, 0]"
    
    # Step 5: Enable all axes
    print("\n6. Enable all axes:")
    dialog.enable_all()
    vector = dialog.get_command_vector()
    print("   Enabled axes: {}".format(dialog.get_enabled()))
    print("   Command vector: {}".format(vector))
    assert all(c for c in dialog.get_enabled()), "All axes should be enabled"
    
    # Step 6: Click "Todos 50%"
    print("\n7. Click 'Todos 50%' with all axes enabled:")
    dialog.home_enabled()
    vector = dialog.get_command_vector()
    print("   Command vector: {}".format(vector))
    assert vector == [50, 50, 50, 50, 50, 50], "All should be 50%"
    
    # Step 7: Disable all axes and verify persistence
    print("\n8. Disable all axes (values persist):")
    dialog.disable_all()
    vector = dialog.get_command_vector()
    print("   Enabled axes: {}".format(dialog.get_enabled()))
    print("   Command vector: {}".format(vector))
    assert vector == [50, 50, 50, 50, 50, 50], "Disabled axes should keep values"
    
    print("\n[OK] Complete axis jog workflow works correctly")


if __name__ == "__main__":
    try:
        print("=" * 60)
        print("COMPLETE AXIS JOG WORKFLOW TEST")
        print("=" * 60)
        
        test_axis_jog_workflow()
        
        print("\n" + "=" * 60)
        print("ALL TESTS PASSED [OK]")
        print("=" * 60)
    except Exception as e:
        print("\n[FAIL] TEST FAILED: {}".format(e))
        import traceback
        traceback.print_exc()
        sys.exit(1)
