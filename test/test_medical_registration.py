"""
test_medical_registration.py
Unit tests for MedicalRegistration transformation, validation, and serialization.
"""

import os
import tempfile
import numpy as np
import pytest

from surgical_sim.medical_registration import MedicalRegistration
from surgical_sim.anatomical_target import AnatomicalTarget


def test_synthetic_table_registration_invariants():
    """Verify synthetic supine table registration satisfies SE(3) properties."""
    reg = MedicalRegistration.create_synthetic_supine_table(patient_center_on_table=(0.0, 0.35, 0.18))
    assert reg.is_simulated

    r = reg.rotation_matrix
    assert np.allclose(r @ r.T, np.eye(3), atol=1e-5)
    assert np.isclose(np.linalg.det(r), 1.0, atol=1e-5)


def test_point_transformation_mm_to_m():
    """Verify conversion from patient mm to robot base meters."""
    reg = MedicalRegistration.create_synthetic_supine_table(patient_center_on_table=(0.0, 0.35, 0.18))

    # Patient origin (0, 0, 0) mm must map to table center (0.0, 0.35, 0.18) m
    p_base = reg.transform_point_mm(0.0, 0.0, 0.0)
    assert np.allclose(p_base, [0.0, 0.35, 0.18], atol=1e-5)


def test_inverse_point_transformation():
    """Verify round-trip mapping from patient to base and back to patient."""
    reg = MedicalRegistration.create_synthetic_supine_table(patient_center_on_table=(0.0, 0.35, 0.18))

    p_orig = (12.5, -8.4, 25.0)  # mm
    p_base = reg.transform_point_mm(*p_orig)  # m
    p_recovered = reg.inverse_transform_point_m(*p_base)  # mm

    assert np.allclose(p_orig, p_recovered, atol=1e-4)


def test_reject_non_orthogonal_matrix():
    """Matrix with non-orthogonal columns must be rejected."""
    bad_mat = np.eye(4)
    bad_mat[0, 1] = 0.5  # Shear / non-orthogonal
    with pytest.raises(ValueError, match="not orthogonal"):
        MedicalRegistration(bad_mat)


def test_reject_reflection_matrix():
    """Matrix with det(R) = -1.0 must be rejected."""
    reflection_mat = np.eye(4)
    reflection_mat[0, 0] = -1.0  # Reflection
    with pytest.raises(ValueError, match="determinant must be \\+1.0"):
        MedicalRegistration(reflection_mat)


def test_yaml_serialization_round_trip():
    """Verify YAML configuration save and reload."""
    reg = MedicalRegistration.create_synthetic_supine_table(patient_center_on_table=(0.05, 0.40, 0.22))

    with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as f:
        yaml_path = f.name

    try:
        reg.save_yaml(yaml_path)
        loaded = MedicalRegistration.load_yaml(yaml_path)
        assert np.allclose(reg.t_matrix, loaded.t_matrix, atol=1e-6)
        assert loaded.is_simulated == reg.is_simulated
    finally:
        if os.path.exists(yaml_path):
            os.remove(yaml_path)

