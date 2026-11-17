"""
test_anatomical_target.py
Unit tests for AnatomicalTargetExtractor centroid, volume, and bounding box extraction.
"""

import shutil
import tempfile
import numpy as np
import pytest

from surgical_sim.dicom_loader import DICOMLoader
from surgical_sim.medical_volume import MedicalVolume
from surgical_sim.medical_segmentation import MedicalSegmentation
from surgical_sim.anatomical_target import AnatomicalTargetExtractor


@pytest.fixture
def target_setup():
    d = tempfile.mkdtemp(prefix="test_anat_")
    gt_center = (4.0, 8.0, 18.0)
    gt_radius = 6.0
    DICOMLoader.generate_synthetic_series(
        output_dir=d,
        num_slices=16,
        rows=48,
        cols=48,
        pixel_spacing=(1.5, 1.5),
        slice_thickness=2.0,
        target_center_mm=gt_center,
        target_radius_mm=gt_radius,
    )
    slices = DICOMLoader.load_series(d)
    vol = MedicalVolume.from_slices(slices)
    yield vol, gt_center, gt_radius
    shutil.rmtree(d, ignore_errors=True)


def test_target_centroid_accuracy(target_setup):
    """Verify extracted centroid matches ground truth location within sub-voxel tolerance."""
    vol, gt_center, gt_radius = target_setup
    seg_res = MedicalSegmentation.segment_target(vol, hu_min=200.0, hu_max=350.0)
    assert seg_res.success

    target = AnatomicalTargetExtractor.extract_from_segmentation(vol, seg_res)
    assert target.valid

    # Centroid should be within 1 mm of ground truth
    err = np.linalg.norm(np.array(target.centroid_patient_mm) - np.array(gt_center))
    assert err < 1.0, f"Centroid error {err:.3f} mm exceeds 1.0 mm tolerance"

    # Equivalent radius should be within 1 mm of ground truth
    radius_err = abs(target.equivalent_radius_mm - gt_radius)
    assert radius_err < 1.0


def test_target_bounding_box(target_setup):
    """Verify physical bounding box encompasses centroid."""
    vol, gt_center, _ = target_setup
    seg_res = MedicalSegmentation.segment_target(vol, hu_min=200.0, hu_max=350.0)
    target = AnatomicalTargetExtractor.extract_from_segmentation(vol, seg_res)

    xmin, xmax, ymin, ymax, zmin, zmax = target.bounding_box_patient_mm
    cx, cy, cz = target.centroid_patient_mm

    assert xmin <= cx <= xmax
    assert ymin <= cy <= ymax
    assert zmin <= cz <= zmax


def test_empty_mask_rejection(target_setup):
    """Empty masks must return valid=False with a clear status message."""
    vol, _, _ = target_setup
    empty_mask = np.zeros((vol.nz, vol.ny, vol.nx), dtype=bool)
    target = AnatomicalTargetExtractor.extract_from_mask(vol, empty_mask)

    assert not target.valid
    assert target.voxel_count == 0
    assert "Empty mask" in target.status_message

