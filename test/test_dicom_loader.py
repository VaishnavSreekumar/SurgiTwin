"""
test_dicom_loader.py
Unit tests for SurgiTwin DICOM Ingestion and Parsing Engine.
"""

import os
import shutil
import tempfile
import numpy as np
import pytest

from surgical_sim.dicom_loader import DICOMLoader, DICOMSlice


@pytest.fixture
def dicom_temp_dir():
    d = tempfile.mkdtemp(prefix="test_dicom_")
    yield d
    shutil.rmtree(d, ignore_errors=True)


def test_synthetic_dicom_generation_and_loading(dicom_temp_dir):
    """Test generating a synthetic DICOM series and loading it."""
    files = DICOMLoader.generate_synthetic_series(
        output_dir=dicom_temp_dir,
        num_slices=8,
        rows=32,
        cols=32,
        pixel_spacing=(2.0, 2.0),
        slice_thickness=3.0,
    )
    assert len(files) == 8

    slices = DICOMLoader.load_series(dicom_temp_dir)
    assert len(slices) == 8

    first = slices[0]
    assert first.rows == 32
    assert first.columns == 32
    assert np.isclose(first.pixel_spacing[0], 2.0)
    assert np.isclose(first.pixel_spacing[1], 2.0)
    assert np.isclose(first.slice_thickness, 3.0)


def test_dicom_physical_slice_sorting(dicom_temp_dir):
    """Slices must sort along normal vector even if filenames are scrambled."""
    files = DICOMLoader.generate_synthetic_series(
        output_dir=dicom_temp_dir,
        num_slices=6,
        rows=16,
        cols=16,
        slice_thickness=2.5,
    )

    # Intentionally rename files to random non-sequential names
    renamed = []
    names = ["z_slice.dcm", "a_slice.dcm", "m_slice.dcm", "b_slice.dcm", "x_slice.dcm", "c_slice.dcm"]
    for old_path, new_name in zip(files, names):
        new_path = os.path.join(dicom_temp_dir, new_name)
        os.rename(old_path, new_path)
        renamed.append(new_path)

    slices = DICOMLoader.load_series(dicom_temp_dir)
    projections = [s.slice_location_projection for s in slices]

    # Must be strictly monotonically increasing along scan normal
    assert np.all(np.diff(projections) > 0.0)


def test_dicom_hu_modality_conversion(dicom_temp_dir):
    """Verify stored raw pixels are correctly rescaled to Hounsfield Units."""
    DICOMLoader.generate_synthetic_series(
        output_dir=dicom_temp_dir,
        num_slices=2,
        rows=32,
        cols=32,
    )
    slices = DICOMLoader.load_series(dicom_temp_dir)
    sl = slices[0]

    # Background air should be ~ -1000 HU
    assert np.isclose(sl.hu_array[0, 0], -1000.0, atol=1.0)

    # Bone shell should be ~ +800 HU
    assert np.isclose(np.max(sl.hu_array), 800.0, atol=1.0)


def test_reject_invalid_dicom_file(dicom_temp_dir):
    """Files lacking DICM magic bytes or corrupted must be rejected."""
    fake_path = os.path.join(dicom_temp_dir, "fake.dcm")
    with open(fake_path, "wb") as f:
        f.write(b"A" * 200)

    with pytest.raises(ValueError, match="Invalid DICOM Part 10 magic bytes"):
        DICOMLoader.load_file(fake_path)

    short_path = os.path.join(dicom_temp_dir, "short.dcm")
    with open(short_path, "wb") as f:
        f.write(b"SHORT")

    with pytest.raises(ValueError, match="File too small"):
        DICOMLoader.load_file(short_path)


def test_reject_inconsistent_series_dimensions(dicom_temp_dir):
    """Series with mismatched slice resolutions must be rejected."""
    DICOMLoader.generate_synthetic_series(dicom_temp_dir, num_slices=2, rows=32, cols=32)

    # Write a 3rd slice with different dimensions (48x48)
    odd_path = os.path.join(dicom_temp_dir, "slice_odd.dcm")
    DICOMLoader._write_part10_file(
        filepath=odd_path,
        instance_number=3,
        rows=48,
        cols=48,
        pixel_spacing=(2.0, 2.0),
        slice_thickness=2.5,
        pos=(0.0, 0.0, 10.0),
        orient=(1.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        slope=1.0,
        intercept=-1024.0,
        pixel_array=np.zeros((48, 48), dtype=np.uint16),
    )

    with pytest.raises(ValueError, match="Inconsistent slice dimensions"):
        DICOMLoader.load_series(dicom_temp_dir)
