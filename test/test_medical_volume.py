"""
test_medical_volume.py
Unit tests for SurgiTwin MedicalVolume 3D reconstruction and coordinate mappings.
"""

import shutil
import tempfile
import numpy as np
import pytest

from surgical_sim.dicom_loader import DICOMLoader
from surgical_sim.medical_volume import MedicalVolume


@pytest.fixture
def volume_fixture():
    d = tempfile.mkdtemp(prefix="test_vol_")
    DICOMLoader.generate_synthetic_series(
        output_dir=d,
        num_slices=10,
        rows=32,
        cols=32,
        pixel_spacing=(1.5, 1.5),
        slice_thickness=2.0,
    )
    slices = DICOMLoader.load_series(d)
    vol = MedicalVolume.from_slices(slices)
    yield vol
    shutil.rmtree(d, ignore_errors=True)


def test_volume_shape_and_spacing(volume_fixture):
    """Verify reconstructed 3D volume dimensions and physical voxel spacing."""
    assert volume_fixture.data.shape == (10, 32, 32)
    assert np.isclose(volume_fixture.spacing_z, 2.0, atol=1e-3)
    assert np.isclose(volume_fixture.spacing_y, 1.5, atol=1e-3)
    assert np.isclose(volume_fixture.spacing_x, 1.5, atol=1e-3)


def test_voxel_to_patient_round_trip(volume_fixture):
    """Verify bidirectional coordinate mapping between voxel grid and physical space."""
    test_voxels = [
        (0, 0, 0),
        (31, 31, 9),
        (16, 16, 5),
        (10.5, 20.2, 3.8),
    ]
    for iv, jv, kv in test_voxels:
        xp, yp, zp = volume_fixture.voxel_to_patient(iv, jv, kv)
        rec_i, rec_j, rec_k = volume_fixture.patient_to_voxel(xp, yp, zp)
        assert np.isclose(rec_i, iv, atol=1e-4)
        assert np.isclose(rec_j, jv, atol=1e-4)
        assert np.isclose(rec_k, kv, atol=1e-4)


def test_orthogonal_slice_extraction(volume_fixture):
    """Verify axial, coronal, and sagittal slice shapes and bounds."""
    # Axial: (Ny, Nx)
    axial = volume_fixture.get_axial_slice(4)
    assert axial.shape == (32, 32)

    # Coronal: (Nz, Nx)
    coronal = volume_fixture.get_coronal_slice(16)
    assert coronal.shape == (10, 32)

    # Sagittal: (Nz, Ny)
    sagittal = volume_fixture.get_sagittal_slice(16)
    assert sagittal.shape == (10, 32)


def test_physical_bounds(volume_fixture):
    """Verify volume physical bounding box calculation."""
    xr, yr, zr = volume_fixture.get_physical_bounds()
    assert xr[1] > xr[0]
    assert yr[1] > yr[0]
    assert zr[1] > zr[0]
    # Total Z extent should be (num_slices - 1) * spacing_z
    assert np.isclose(zr[1] - zr[0], 9 * 2.0, atol=1e-3)


def test_reject_insufficient_slices(volume_fixture):
    """Reconstruction requires at least 2 slices."""
    with pytest.raises(ValueError, match="Cannot reconstruct volume from empty slice list"):
        MedicalVolume.from_slices([])

    # Mock single slice
    from surgical_sim.dicom_loader import DICOMSlice
    single_slice = DICOMSlice(
        filepath="dummy.dcm",
        instance_number=1,
        rows=10,
        columns=10,
        pixel_spacing=(1.0, 1.0),
        slice_thickness=1.0,
        image_position=(0.0, 0.0, 0.0),
        image_orientation=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        rescale_slope=1.0,
        rescale_intercept=0.0,
        pixel_array=np.zeros((10, 10)),
        hu_array=np.zeros((10, 10)),
    )
    with pytest.raises(ValueError, match="At least 2 slices required"):
        MedicalVolume.from_slices([single_slice])
