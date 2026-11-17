"""
test_medical_segmentation.py
Unit tests for MedicalSegmentation thresholding, connected components, and STL export.
"""

import os
import shutil
import tempfile
import numpy as np
import pytest

from surgical_sim.dicom_loader import DICOMLoader
from surgical_sim.medical_volume import MedicalVolume
from surgical_sim.medical_segmentation import MedicalSegmentation


@pytest.fixture
def volume_with_target():
    d = tempfile.mkdtemp(prefix="test_seg_")
    DICOMLoader.generate_synthetic_series(
        output_dir=d,
        num_slices=12,
        rows=48,
        cols=48,
        pixel_spacing=(1.5, 1.5),
        slice_thickness=2.5,
        target_center_mm=(0.0, 5.0, 15.0),
        target_radius_mm=7.0,
    )
    slices = DICOMLoader.load_series(d)
    vol = MedicalVolume.from_slices(slices)
    yield vol
    shutil.rmtree(d, ignore_errors=True)


def test_thresholding(volume_with_target):
    """Verify thresholding selects voxels in specified HU range."""
    # Bone shell is ~800 HU
    bone_mask = MedicalSegmentation.threshold(volume_with_target, hu_min=700.0, hu_max=900.0)
    assert np.any(bone_mask)
    assert np.all(volume_with_target.data[bone_mask] >= 700.0)
    assert np.all(volume_with_target.data[bone_mask] <= 900.0)


def test_segment_target_extraction(volume_with_target):
    """Segment target lesion (~280 HU) from anatomical volume."""
    res = MedicalSegmentation.segment_target(
        volume_with_target,
        hu_min=200.0,
        hu_max=350.0,
        min_volume_mm3=50.0,
    )
    assert res.success
    assert res.voxel_count > 0
    assert res.volume_mm3 > 50.0

    # Expected volume of 7 mm sphere: 4/3 * pi * 7^3 = ~1436 mm^3
    assert 1000.0 <= res.volume_mm3 <= 2000.0


def test_empty_threshold_handling(volume_with_target):
    """Non-existent intensity values return success=False gracefully."""
    res = MedicalSegmentation.segment_target(
        volume_with_target,
        hu_min=2000.0,
        hu_max=3000.0,
    )
    assert not res.success
    assert res.voxel_count == 0
    assert "No voxels matched" in res.status_message


def test_export_stl(volume_with_target):
    """Verify surface export creates a valid non-empty STL file."""
    res = MedicalSegmentation.segment_target(volume_with_target, hu_min=200.0, hu_max=350.0)
    assert res.success

    with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as f:
        stl_path = f.name

    try:
        MedicalSegmentation.export_stl(
            mask=res.mask,
            voxel_spacing=(volume_with_target.spacing_z, volume_with_target.spacing_y, volume_with_target.spacing_x),
            filepath=stl_path,
        )
        assert os.path.exists(stl_path)
        assert os.path.getsize(stl_path) > 100
    finally:
        if os.path.exists(stl_path):
            os.remove(stl_path)

