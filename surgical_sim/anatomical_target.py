"""
anatomical_target.py
SurgiTwin Anatomical Target Extraction Engine.

Calculates geometric centroids, physical DICOM coordinates (LPS mm),
equivalent lesion radii, and spatial extents from segmented anatomical structures.
"""

from dataclasses import dataclass
import math
from typing import Optional, Tuple
import numpy as np

from surgical_sim.medical_volume import MedicalVolume
from surgical_sim.medical_segmentation import SegmentationResult


@dataclass
class AnatomicalTarget:
    """Structure encapsulating an extracted anatomical target."""
    valid: bool
    centroid_voxel: Tuple[float, float, float]                       # (i_col, j_row, k_slice)
    centroid_patient_mm: Tuple[float, float, float]                  # (X, Y, Z) in mm (DICOM LPS)
    volume_mm3: float
    equivalent_radius_mm: float
    bounding_box_patient_mm: Tuple[float, float, float, float, float, float]  # (xmin, xmax, ymin, ymax, zmin, zmax)
    voxel_count: int
    status_message: str


class AnatomicalTargetExtractor:
    """
    Extracts geometric target parameters from segmented medical volumes.
    """

    @classmethod
    def extract_from_mask(
        cls,
        volume: MedicalVolume,
        mask: np.ndarray,
    ) -> AnatomicalTarget:
        """
        Extract anatomical target centroid and metrics from a 3D binary mask.

        :param volume: MedicalVolume instance containing spatial coordinate mapping.
        :param mask: 3D boolean array of shape (Nz, Ny, Nx).
        :return: AnatomicalTarget dataclass.
        """
        if mask.shape != (volume.nz, volume.ny, volume.nx):
            raise ValueError(f"Mask shape {mask.shape} does not match volume shape {(volume.nz, volume.ny, volume.nx)}")

        k_indices, j_indices, i_indices = np.where(mask)
        n_voxels = len(k_indices)

        if n_voxels == 0:
            return AnatomicalTarget(
                valid=False,
                centroid_voxel=(0.0, 0.0, 0.0),
                centroid_patient_mm=(0.0, 0.0, 0.0),
                volume_mm3=0.0,
                equivalent_radius_mm=0.0,
                bounding_box_patient_mm=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                voxel_count=0,
                status_message="Empty mask: no anatomical structure detected",
            )

        # Centroid in voxel coordinates (i_col, j_row, k_slice)
        c_i = float(np.mean(i_indices))
        c_j = float(np.mean(j_indices))
        c_k = float(np.mean(k_indices))

        # Convert centroid to DICOM patient physical coordinates (mm)
        p_centroid = volume.voxel_to_patient(c_i, c_j, c_k)

        # Physical volume in mm^3
        voxel_vol = volume.spacing_z * volume.spacing_y * volume.spacing_x
        tot_vol_mm3 = float(n_voxels * voxel_vol)

        # Equivalent sphere radius R = (3V / 4pi)^(1/3)
        equiv_radius = float((3.0 * tot_vol_mm3 / (4.0 * math.pi)) ** (1.0 / 3.0))

        # Physical bounding box in mm
        corner_voxels = [
            (np.min(i_indices), np.min(j_indices), np.min(k_indices)),
            (np.max(i_indices), np.min(j_indices), np.min(k_indices)),
            (np.min(i_indices), np.max(j_indices), np.min(k_indices)),
            (np.max(i_indices), np.max(j_indices), np.min(k_indices)),
            (np.min(i_indices), np.min(j_indices), np.max(k_indices)),
            (np.max(i_indices), np.min(j_indices), np.max(k_indices)),
            (np.min(i_indices), np.max(j_indices), np.max(k_indices)),
            (np.max(i_indices), np.max(j_indices), np.max(k_indices)),
        ]
        pts_mm = np.array([volume.voxel_to_patient(*cv) for cv in corner_voxels])
        min_p = np.min(pts_mm, axis=0)
        max_p = np.max(pts_mm, axis=0)

        bbox_patient = (
            float(min_p[0]), float(max_p[0]),
            float(min_p[1]), float(max_p[1]),
            float(min_p[2]), float(max_p[2]),
        )

        return AnatomicalTarget(
            valid=True,
            centroid_voxel=(c_i, c_j, c_k),
            centroid_patient_mm=p_centroid,
            volume_mm3=tot_vol_mm3,
            equivalent_radius_mm=equiv_radius,
            bounding_box_patient_mm=bbox_patient,
            voxel_count=n_voxels,
            status_message=f"Target extracted at {np.round(p_centroid, 2)} mm (Vol: {tot_vol_mm3:.1f} mm^3, R: {equiv_radius:.1f} mm)",
        )

    @classmethod
    def extract_from_segmentation(
        cls,
        volume: MedicalVolume,
        seg_result: SegmentationResult,
    ) -> AnatomicalTarget:
        """
        Extract anatomical target directly from a SegmentationResult object.
        """
        if not seg_result.success:
            return AnatomicalTarget(
                valid=False,
                centroid_voxel=(0.0, 0.0, 0.0),
                centroid_patient_mm=(0.0, 0.0, 0.0),
                volume_mm3=0.0,
                equivalent_radius_mm=0.0,
                bounding_box_patient_mm=(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                voxel_count=0,
                status_message=f"Segmentation failed: {seg_result.status_message}",
            )

        return cls.extract_from_mask(volume, seg_result.mask)

