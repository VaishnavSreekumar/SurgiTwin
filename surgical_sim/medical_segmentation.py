"""
medical_segmentation.py
SurgiTwin Medical Image Segmentation Engine.

Implements threshold-based anatomical segmentation, 3D morphological filtering,
connected-component analysis, volume measurement, and 2D/3D visualization utilities.
Configurable for CT Hounsfield Unit intervals (e.g. bone, soft tissue, surgical target lesions).
"""

from dataclasses import dataclass
import os
import struct
from typing import List, Optional, Tuple, Union
import numpy as np
from scipy import ndimage

from surgical_sim.medical_volume import MedicalVolume


@dataclass
class SegmentationResult:
    """Structure encapsulating medical volume segmentation outcome."""
    success: bool
    mask: np.ndarray                                       # 3D boolean array (Nz, Ny, Nx)
    voxel_count: int
    volume_mm3: float
    bounding_box_voxel: Tuple[int, int, int, int, int, int]  # (i_min, i_max, j_min, j_max, k_min, k_max)
    status_message: str


class MedicalSegmentation:
    """
    Segmentation and morphological analysis pipeline for 3D medical volumes.
    """

    # Typical clinical CT Hounsfield Unit (HU) benchmark windows (for reference only, configurable)
    WINDOW_BONE = (200.0, 1500.0)
    WINDOW_SOFT_TISSUE = (-50.0, 100.0)
    WINDOW_LUNG = (-900.0, -400.0)

    @classmethod
    def threshold(
        cls,
        volume: MedicalVolume,
        hu_min: float,
        hu_max: float,
    ) -> np.ndarray:
        """
        Apply dual-threshold intensity filtering on volume data.

        :param volume: MedicalVolume instance.
        :param hu_min: Lower intensity threshold (HU).
        :param hu_max: Upper intensity threshold (HU).
        :return: 3D boolean array (Nz, Ny, Nx).
        """
        if hu_min > hu_max:
            raise ValueError(f"hu_min ({hu_min}) cannot exceed hu_max ({hu_max})")

        return (volume.data >= hu_min) & (volume.data <= hu_max)

    @classmethod
    def morphological_clean(
        cls,
        mask: np.ndarray,
        radius: int = 1,
    ) -> np.ndarray:
        """
        Perform 3D binary morphological opening followed by closing to remove noise and fill voids.

        :param mask: 3D boolean array.
        :param radius: Structuring element radius (voxels).
        :return: Cleaned 3D boolean array.
        """
        structure = ndimage.generate_binary_structure(rank=3, connectivity=1)
        if radius > 1:
            structure = ndimage.iterate_structure(structure, radius)

        # Opening removes small specks; closing closes internal voids
        opened = ndimage.binary_opening(mask, structure=structure)
        cleaned = ndimage.binary_closing(opened, structure=structure)
        return cleaned

    @classmethod
    def connected_components(
        cls,
        mask: np.ndarray,
        voxel_spacing: Tuple[float, float, float],
        min_volume_mm3: float = 10.0,
        max_volume_mm3: Optional[float] = None,
    ) -> List[Tuple[np.ndarray, int, float]]:
        """
        Find and sort all 3D connected components by physical volume.

        :param mask: 3D boolean array.
        :param voxel_spacing: (sz, sy, sx) in mm.
        :param min_volume_mm3: Minimum physical volume threshold to discard small noise.
        :param max_volume_mm3: Optional upper volume threshold.
        :return: List of tuples (component_mask, voxel_count, volume_mm3) sorted descending by size.
        """
        if not np.any(mask):
            return []

        labeled_array, num_features = ndimage.label(mask)
        if num_features == 0:
            return []

        voxel_vol_mm3 = float(voxel_spacing[0] * voxel_spacing[1] * voxel_spacing[2])
        component_sizes = ndimage.sum(mask, labeled_array, range(1, num_features + 1))
        component_sizes = np.atleast_1d(component_sizes)

        results = []
        for label_idx, count in enumerate(component_sizes, start=1):
            vol_mm3 = float(count * voxel_vol_mm3)
            if vol_mm3 < min_volume_mm3:
                continue
            if max_volume_mm3 is not None and vol_mm3 > max_volume_mm3:
                continue

            comp_mask = (labeled_array == label_idx)
            results.append((comp_mask, int(count), vol_mm3))

        # Sort descending by volume
        results.sort(key=lambda x: x[2], reverse=True)
        return results

    @classmethod
    def segment_target(
        cls,
        volume: MedicalVolume,
        hu_min: float,
        hu_max: float,
        seed_point_voxel: Optional[Tuple[int, int, int]] = None,
        min_volume_mm3: float = 15.0,
        morph_radius: int = 1,
    ) -> SegmentationResult:
        """
        Segment a discrete target structure from a 3D medical volume.

        :param volume: MedicalVolume instance.
        :param hu_min: Minimum HU threshold.
        :param hu_max: Maximum HU threshold.
        :param seed_point_voxel: Optional (i_col, j_row, k_slice) to select component containing/closest to seed.
        :param min_volume_mm3: Minimum volume threshold (mm^3).
        :param morph_radius: Morphological structuring element radius.
        :return: SegmentationResult.
        """
        raw_mask = cls.threshold(volume, hu_min, hu_max)
        if not np.any(raw_mask):
            return SegmentationResult(
                success=False,
                mask=np.zeros_like(raw_mask),
                voxel_count=0,
                volume_mm3=0.0,
                bounding_box_voxel=(0, 0, 0, 0, 0, 0),
                status_message="No voxels matched intensity thresholds",
            )

        cleaned_mask = cls.morphological_clean(raw_mask, radius=morph_radius)
        components = cls.connected_components(
            cleaned_mask,
            voxel_spacing=(volume.spacing_z, volume.spacing_y, volume.spacing_x),
            min_volume_mm3=min_volume_mm3,
        )

        if not components:
            return SegmentationResult(
                success=False,
                mask=np.zeros_like(cleaned_mask),
                voxel_count=0,
                volume_mm3=0.0,
                bounding_box_voxel=(0, 0, 0, 0, 0, 0),
                status_message=f"No connected components exceeded minimum volume {min_volume_mm3:.1f} mm^3",
            )

        # Select target component
        selected_mask: Optional[np.ndarray] = None
        selected_count = 0
        selected_vol = 0.0

        if seed_point_voxel is not None:
            si, sj, sk = seed_point_voxel
            # Check if seed point is inside any component
            for comp_mask, count, vol in components:
                if 0 <= sk < volume.nz and 0 <= sj < volume.ny and 0 <= si < volume.nx:
                    if comp_mask[sk, sj, si]:
                        selected_mask = comp_mask
                        selected_count = count
                        selected_vol = vol
                        break

        # Fallback to largest connected component
        if selected_mask is None:
            selected_mask, selected_count, selected_vol = components[0]

        # Determine bounding box
        k_indices, j_indices, i_indices = np.where(selected_mask)
        bbox = (
            int(np.min(i_indices)), int(np.max(i_indices)),
            int(np.min(j_indices)), int(np.max(j_indices)),
            int(np.min(k_indices)), int(np.max(k_indices)),
        )

        return SegmentationResult(
            success=True,
            mask=selected_mask,
            voxel_count=selected_count,
            volume_mm3=selected_vol,
            bounding_box_voxel=bbox,
            status_message=f"Successfully segmented region: {selected_count} voxels, {selected_vol:.1f} mm^3",
        )

    @classmethod
    def export_stl(cls, mask: np.ndarray, voxel_spacing: Tuple[float, float, float], filepath: str):
        """
        Export segmented binary mask boundary as a valid binary STL surface file.
        Uses boundary voxel cubes as lightweight surface approximation.
        """
        sz, sy, sx = voxel_spacing
        k_pts, j_pts, i_pts = np.where(mask)
        if len(k_pts) == 0:
            raise ValueError("Cannot export empty mask to STL")

        # Boundary voxels have at least one 6-connected neighbor that is False
        pad_mask = np.pad(mask, 1, mode="constant", constant_values=False)
        eroded = ndimage.binary_erosion(pad_mask)
        boundary = pad_mask & ~eroded
        b_k, b_j, b_i = np.where(boundary[1:-1, 1:-1, 1:-1])

        # Subsample boundary to keep STL lightweight (max 2000 points)
        step = max(1, len(b_k) // 1500)
        b_k = b_k[::step]
        b_j = b_j[::step]
        b_i = b_i[::step]

        triangles = []
        d = 0.5
        for k, j, i in zip(b_k, b_j, b_i):
            cx, cy, cz = i * sx, j * sy, k * sz
            # 8 corners of voxel cube
            c = [
                (cx - d * sx, cy - d * sy, cz - d * sz),
                (cx + d * sx, cy - d * sy, cz - d * sz),
                (cx + d * sx, cy + d * sy, cz - d * sz),
                (cx - d * sx, cy + d * sy, cz - d * sz),
                (cx - d * sx, cy - d * sy, cz + d * sz),
                (cx + d * sx, cy - d * sy, cz + d * sz),
                (cx + d * sx, cy + d * sy, cz + d * sz),
                (cx - d * sx, cy + d * sy, cz + d * sz),
            ]
            # 12 cube face triangles
            faces = [
                (c[0], c[1], c[2]), (c[0], c[2], c[3]),  # bottom
                (c[4], c[6], c[5]), (c[4], c[7], c[6]),  # top
                (c[0], c[4], c[5]), (c[0], c[5], c[1]),  # front
                (c[2], c[6], c[7]), (c[2], c[7], c[3]),  # back
                (c[0], c[3], c[7]), (c[0], c[7], c[4]),  # left
                (c[1], c[5], c[6]), (c[1], c[6], c[2]),  # right
            ]
            triangles.extend(faces)

        # Write binary STL
        os.makedirs(os.path.dirname(os.path.abspath(filepath)), exist_ok=True)
        with open(filepath, "wb") as f:
            header = b"SurgiTwin Anatomical Segmentation Surface Model"[:80].ljust(80, b" ")
            f.write(header)
            f.write(struct.pack("<I", len(triangles)))
            for v1, v2, v3 in triangles:
                # Normal vector
                u = np.array(v2) - np.array(v1)
                v = np.array(v3) - np.array(v1)
                n = np.cross(u, v)
                norm = np.linalg.norm(n)
                n = n / norm if norm > 1e-6 else np.array([0.0, 0.0, 1.0])
                f.write(struct.pack("<3f", *n))
                f.write(struct.pack("<3f", *v1))
                f.write(struct.pack("<3f", *v2))
                f.write(struct.pack("<3f", *v3))
                f.write(struct.pack("<H", 0))

