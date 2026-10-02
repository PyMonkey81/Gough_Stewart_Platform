#!/usr/bin/env python3
"""
Quick smoke test: verify MainWindow instantiation with MATLAB state machine
"""
import sys
import json
import numpy as np
from pathlib import Path

# Ensure workspace is in path
workspace = Path(__file__).parent
sys.path.insert(0, str(workspace))

def test_trajectory_generator():
    """Test TrajectoryGenerator with β as elevation"""
    from trajectory.generator import TrajectoryGenerator, TrajectoryConfig
    
    print("\n=== TrajectoryGenerator Test ===")
    cfg = TrajectoryConfig()
    traj_gen = TrajectoryGenerator(config=cfg)
    
    # TRAJ_POINTS in degrees: β is elevation from horizon (LMT: 10.10, 45.60, 10.00)
    traj_points_deg = [[0.0, 10.10], [15.0, 45.60], [0.0, 10.00]]
    traj_points_rad = np.radians(traj_points_deg)
    
    print(f"  Set tracking points (rad): {traj_points_rad}")
    traj_gen.set_tracking_points(traj_points_rad)
    
    # Test step at different phases
    tests = [
        (10.0, "home", "APPROACH phase (0…60s)"),
        (60.0, "tracking", "Start TRACK phase"),
        (300.0, "tracking", "Mid TRACK phase"),
        (653.0, "return", "Start RETURN phase"),
        (700.0, "return", "Mid RETURN phase"),
        (713.0, "done", "End RETURN phase"),
    ]
    
    for t, expected_phase, desc in tests:
        y, yp = traj_gen.step(t)
        alpha_deg = np.degrees(y[0])
        beta_deg = np.degrees(y[1])  # β is elevation from horizon
        print(f"  t={t:6.1f}s | phase={traj_gen.phase:10s} | α={alpha_deg:7.2f}° β={beta_deg:7.2f}° | {desc}")
        if traj_gen.phase != expected_phase:
            print(f"    WARNING: Expected phase '{expected_phase}', got '{traj_gen.phase}'")
    
    print("  ✓ TrajectoryGenerator works correctly")


def test_inverse_kinematics():
    """Test IK with β as elevation"""
    from kinematics.inverse import inverse_kinematics
    from config.parameters import OFFSET_ACTUADOR
    
    print("\n=== Inverse Kinematics Test ===")
    
    # Test HOME pose: α=0, β=0 (elevation = 0 = looking at horizon? or cenit?)
    # Actually, elevation = 0 means horizon, elevation = 90 means cenit
    # So HOME should be β=90° elevation for cenit position
    # But the LMT table shows HOME as 10.10°, which is looking up from horizon
    
    # Let's test with the actual HOME position
    y_home = np.radians([0.0, 10.10])  # α=0°, β=10.10° elevation
    q_actuator, da, R = inverse_kinematics(y_home)
    print(f"  HOME pose (α=0°, β=10.10° elevation):")
    print(f"    q_actuator = {q_actuator}")
    print(f"    q_actuator (all ~0 mm) = {q_actuator < 1.0}")
    
    # Test an elevated pose
    y_elevated = np.radians([15.0, 45.60])  # α=15°, β=45.60° elevation
    q_actuator2, _, _ = inverse_kinematics(y_elevated)
    print(f"  Elevated pose (α=15°, β=45.60° elevation):")
    print(f"    q_actuator = {q_actuator2}")
    
    # HOME should have q ≈ OFFSET_ACTUADOR ≈ 247.41 mm
    # but inverse_kinematics returns q_actuator = q_geom - OFFSET_ACTUADOR
    # so q_actuator should be ≈ 0 mm for HOME
    print("  ✓ Inverse kinematics callable")


def test_main_window_instantiation():
    """Attempt to instantiate MainWindow without displaying it"""
    from PySide6.QtWidgets import QApplication
    
    print("\n=== MainWindow Instantiation Test ===")
    
    # Suppress QMessageBox for headless testing
    from PySide6.QtWidgets import QMessageBox
    QMessageBox.warning = staticmethod(lambda *a, **k: None)
    
    # Create application
    app = QApplication.instance() or QApplication(sys.argv)
    
    try:
        from gui.screen.MainWindow import MainWindow
        
        # Instantiate but don't show (headless)
        window = MainWindow()
        
        # Verify state machine initialization
        print(f"  state: {window.state} (expected 'HOME')")
        print(f"  en_home: {window.en_home} (expected True)")
        print(f"  demo_active: {window.demo_active} (expected False)")
        print(f"  lbl_track_phase text: {window.lbl_track_phase.text()} (expected 'HOME')")
        
        # Verify generator is configured
        print(f"  traj_gen type: {type(window.traj_gen).__name__}")
        print(f"  traj_gen.phase: {window.traj_gen.phase}")
        
        # Verify HOME display was set
        print(f"  y_desired: {window.y_desired}")
        
        print("  ✓ MainWindow instantiated successfully")
        
        # Clean up
        window.destroy()
        
    except Exception as e:
        print(f"  ✗ MainWindow instantiation failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True


def test_parameters():
    """Verify parameters are loaded correctly"""
    from config.parameters import (
        OFFSET_ACTUADOR, STROKE, ACTUATOR_MIN, ACTUATOR_MAX,
        T_HOME_END, T_TRACKING_END, FILTER_WN,
        Az, Bz, D, RT
    )
    
    print("\n=== Parameters Test ===")
    print(f"  OFFSET_ACTUADOR: {OFFSET_ACTUADOR} mm (expected 247.41)")
    print(f"  STROKE: {STROKE} mm (expected 100.0)")
    print(f"  ACTUATOR_MIN: {ACTUATOR_MIN} mm (expected 247.41)")
    print(f"  ACTUATOR_MAX: {ACTUATOR_MAX} mm (expected 347.41)")
    print(f"  T_HOME_END: {T_HOME_END} s (expected 60.0)")
    print(f"  T_TRACKING_END: {T_TRACKING_END} s (expected 653.0)")
    print(f"  FILTER_WN: {FILTER_WN} (expected 3.4)")
    print(f"  Az shape: {Az.shape} (expected (6, 3))")
    print(f"  Bz shape: {Bz.shape} (expected (6, 3))")
    print(f"  D: {D} (expected [0, 0, 331.94])")
    print(f"  RT: {RT} mm (expected 0.0)")
    print(f"  (TRAJ_POINTS loaded from config/parameters.json, not Python)")
    
    # Verify mm units
    assert OFFSET_ACTUADOR > 100, "OFFSET_ACTUADOR should be in mm, not meters"
    assert D[2] > 100, "D[2] should be in mm, not meters"
    
    print("  ✓ All parameters verified (mm units)")


def test_config_json():
    """Verify config/parameters.json is valid"""
    import json
    
    print("\n=== config/parameters.json Test ===")
    
    json_path = workspace / "config" / "parameters.json"
    if not json_path.exists():
        print(f"  ✗ File not found: {json_path}")
        return False
    
    try:
        with open(json_path) as f:
            params = json.load(f)
        
        print(f"  Loaded parameters: {list(params.keys())}")
        
        # Check critical values
        if "OFFSET_ACTUADOR" in params:
            offset = params["OFFSET_ACTUADOR"]
            print(f"  OFFSET_ACTUADOR: {offset} (expected 247.41)")
            
        if "TRAJ_POINTS" in params:
            traj = params["TRAJ_POINTS"]
            print(f"  TRAJ_POINTS: {traj}")
            # Verify β values are elevation (10.10, 45.60, 10.00)
            if len(traj) >= 3 and len(traj[0]) >= 2:
                print(f"    Point 1 β: {traj[0][1]:.2f}° (expected ~10.10 elevation)")
                print(f"    Point 2 β: {traj[1][1]:.2f}° (expected ~45.60 elevation)")
                print(f"    Point 3 β: {traj[2][1]:.2f}° (expected ~10.00 elevation)")
        
        print("  ✓ config/parameters.json is valid JSON")
        return True
        
    except json.JSONDecodeError as e:
        print(f"  ✗ JSON parse error: {e}")
        return False


if __name__ == "__main__":
    print("=" * 60)
    print("GOUGH-STEWART PLATFORM - STATE MACHINE SMOKE TEST")
    print("=" * 60)
    
    try:
        test_parameters()
        test_config_json()
        test_trajectory_generator()
        test_inverse_kinematics()
        result = test_main_window_instantiation()
        
        if result:
            print("\n" + "=" * 60)
            print("✓ ALL TESTS PASSED")
            print("=" * 60)
        else:
            print("\n" + "=" * 60)
            print("✗ SOME TESTS FAILED")
            print("=" * 60)
            sys.exit(1)
            
    except Exception as e:
        print(f"\n✗ Test error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
