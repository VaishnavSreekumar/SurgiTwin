"""
medical_volume.py
SurgiTwin 3D Medical Volume Reconstruction Engine.

Reconstructs 3D volumetric representations from sorted DICOM slice series,
establishes rigorous affine mappings between voxel grid indices (i, j, k)
and physical DICOM Patient coordinates (LPS convention: Left-Posterior-Superior),
and provides Multi-Planar Reconstruction (MPR) orthogonal slice extraction.
"""

from typing import List, Optional, Tuple, Union
import numpy as np

from surgical_sim.dicom_loader import DICOMSlice


class MedicalVolume:
    """
    3D Medical Imaging Volume representation.

    Coordinate system:
    - Voxel grid indices: (k_z, j_y, i_x) in NumPy array volume[k, j, i]
      - k_z in [0, N_z - 1]: slice index along scan normal
      - j_y in [0, N_y - 1]: row index along column direction
      - i_x in [0, N_x - 1]: column index along row direction
    - Physical coordinates: DICOM Patient Coordinate System (LPS, millimeters)
      - +X: towards patient Left
      - +Y: towards patient Posterior
      - +Z: towards patient Superior
    """

    def __init__(
        self,
        volume_data: np.ndarray,
        voxel_spacing: Tuple[float, float, float],
        origin: Tuple[float, float, float],
        orientation: Tuple[float, float, float, float, float, float],
    ):
        """
        Initialize medical volume.

        :param volume_data: 3D numpy array of shape (N_z, N_y, N_x) in Hounsfield Units.
        :param voxel_spacing: (s_z, s_y, s_x) in mm.
        :param origin: (X_0, Y_0, Z_0) physical coordinates of voxel (0, 0, 0) in mm.
        :param orientation: (r_xx, r_xy, r_xz, c_yx, c_yy, c_yz) 6 direction cosines.
        """
        self.data = np.asarray(volume_data, dtype=np.float32)
        if self.data.ndim != 3:
            raise ValueError(f"Volume data must be 3-dimensional, got {self.data.ndim}D")

        self.nz, self.ny, self.nx = self.data.shape
        self.spacing_z, self.spacing_y, self.spacing_x = [float(v) for v in voxel_spacing]
        self.origin = np.array(origin, dtype=np.float64)

        r_row = np.array(orientation[:3], dtype=np.float64)
        r_col = np.array(orientation[3:], dtype=np.float64)

        # Ensure unit vectors
        r_row = r_row / (np.linalg.norm(r_row) + 1e-12)
        r_col = r_col / (np.linalg.norm(r_col) + 1e-12)

        # Slice normal n = row x col
        r_normal = np.cross(r_row, r_col)
        r_normal = r_normal / (np.linalg.norm(r_normal) + 1e-12)

        self.r_row = r_row
        self.r_col = r_col
        self.r_normal = r_normal
        self.orientation = tuple(r_row.tolist() + r_col.tolist())

        # Construct 4x4 affine matrix from voxel index [i_x, j_y, k_z, 1]^T to patient [X, Y, Z, 1]^T
        # Patient_pos = origin + i * s_x * r_row + j * s_y * r_col + k * s_z * r_normal
        self.affine_voxel_to_patient = np.eye(4, dtype=np.float64)
        self.affine_voxel_to_patient[:3, 0] = self.r_row * self.spacing_x
        self.affine_voxel_to_patient[:3, 1] = self.r_col * self.spacing_y
        self.affine_voxel_to_patient[:3, 2] = self.r_normal * self.spacing_z
        self.affine_voxel_to_patient[:3, 3] = self.origin

        self.affine_patient_to_voxel = np.linalg.inv(self.affine_voxel_to_patient)

    @classmethod
    def from_slices(cls, slices: List[DICOMSlice]) -> "MedicalVolume":
        """
        Reconstruct a 3D medical volume from a physically sorted list of DICOMSlice objects.
        """
        if not slices:
            raise ValueError("Cannot reconstruct volume from empty slice list")
        if len(slices) < 2:
            raise ValueError(f"At least 2 slices required for 3D reconstruction, got {len(slices)}")

        ref = slices[0]
        nz = len(slices)
        ny = ref.rows
        nx = ref.columns

        spacing_y, spacing_x = ref.pixel_spacing

        # Determine inter-slice spacing along scan normal
        positions = np.array([s.image_position for s in slices], dtype=np.float64)
        r_row = np.array(ref.image_orientation[:3], dtype=np.float64)
        r_col = np.array(ref.image_orientation[3:], dtype=np.float64)
        normal = np.cross(r_row, r_col)
        normal = normal / (np.linalg.norm(normal) + 1e-12)

        projections = np.dot(positions, normal)
        deltas = np.diff(projections)

        if np.any(np.abs(deltas) < 1e-4):
            raise ValueError("Duplicate or overlapping slice positions detected")

        avg_dz = float(np.mean(np.abs(deltas)))

        # Stack into 3D volume array (N_z, N_y, N_x)
        vol_arr = np.zeros((nz, ny, nx), dtype=np.float32)
        for k, sl in enumerate(slices):
            vol_arr[k, :, :] = sl.hu_array

        origin = slices[0].image_position
        orient = ref.image_orientation

        return cls(
            volume_data=vol_arr,
            voxel_spacing=(avg_dz, spacing_y, spacing_x),
            origin=origin,
            orientation=orient,
        )

    def voxel_to_patient(self, i_x: float, j_y: float, k_z: float) -> Tuple[float, float, float]:
        """
        Convert continuous voxel coordinates (i_col, j_row, k_slice) to physical DICOM coordinates (mm).
        """
        v_h = np.array([float(i_x), float(j_y), float(k_z), 1.0], dtype=np.float64)
        p_h = self.affine_voxel_to_patient @ v_h
        return (float(p_h[0]), float(p_h[1]), float(p_h[2]))

    def patient_to_voxel(self, x_mm: float, y_mm: float, z_mm: float) -> Tuple[float, float, float]:
        """
        Convert physical DICOM coordinates (mm) to continuous voxel coordinates (i_col, j_row, k_slice).
        """
        p_h = np.array([float(x_mm), float(y_mm), float(z_mm), 1.0], dtype=np.float64)
        v_h = self.affine_patient_to_voxel @ p_h
        return (float(v_h[0]), float(v_h[1]), float(v_h[2]))

    def get_axial_slice(self, k: int) -> np.ndarray:
        """Extract axial slice (k index, shape: Ny x Nx)."""
        idx = max(0, min(int(k), self.nz - 1))
        return self.data[idx, :, :].copy()

    def get_coronal_slice(self, j: int) -> np.ndarray:
        """Extract coronal slice (j index, shape: Nz x Nx)."""
        idx = max(0, min(int(j), self.ny - 1))
        return self.data[:, idx, :].copy()

    def get_sagittal_slice(self, i: int) -> np.ndarray:
        """Extract sagittal slice (i index, shape: Nz x Ny)."""
        idx = max(0, min(int(i), self.nx - 1))
        return self.data[:, :, idx].copy()

    def get_physical_bounds(self) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        """
        Compute minimum and maximum physical bounds (X_range, Y_range, Z_range) in mm.
        """
        corners_voxel = [
            (0, 0, 0),
            (self.nx - 1, 0, 0),
            (0, self.ny - 1, 0),
            (self.nx - 1, self.ny - 1, 0),
            (0, 0, self.nz - 1),
            (self.nx - 1, 0, self.nz - 1),
            (0, self.ny - 1, self.nz - 1),
            (self.nx - 1, self.ny - 1, self.nz - 1),
        ]
        pts = np.array([self.voxel_to_patient(*c) for c in corners_voxel])
        min_p = np.min(pts, axis=0)
        max_p = np.max(pts, axis=0)

        return (
            (float(min_p[0]), float(max_p[0])),
            (float(min_p[1]), float(max_p[1])),
            (float(min_p[2]), float(max_p[2])),
        )

