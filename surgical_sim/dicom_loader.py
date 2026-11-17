"""
dicom_loader.py
SurgiTwin Modular DICOM Ingestion and Parsing Engine.

Supports loading standard DICOM Part 10 medical image series (CT/MRI),
metadata extraction (pixel spacing, slice thickness, patient position, orientation),
physical position slice sorting along the scan normal vector,
Hounsfield Unit (HU) modality rescaling, and synthetic DICOM dataset generation.
Includes a native DICOM Part 10 parser with fallback/interoperability with pydicom.
"""

from dataclasses import dataclass
import os
import struct
from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np

# Optional pydicom integration
try:
    import pydicom
    HAS_PYDICOM = True
except ImportError:
    HAS_PYDICOM = False


@dataclass
class DICOMSlice:
    """Structure encapsulating a single DICOM cross-sectional image slice."""
    filepath: str
    instance_number: int
    rows: int
    columns: int
    pixel_spacing: Tuple[float, float]               # (spacing_y, spacing_x) in mm
    slice_thickness: float                          # mm
    image_position: Tuple[float, float, float]       # (X, Y, Z) in mm (patient physical frame)
    image_orientation: Tuple[float, float, float, float, float, float]  # 6 direction cosines
    rescale_slope: float
    rescale_intercept: float
    pixel_array: np.ndarray                          # Raw stored pixel data
    hu_array: np.ndarray                             # Rescaled values (e.g. Hounsfield Units for CT)
    slice_location_projection: float = 0.0           # Position along normal vector for physical sorting


class DICOMLoader:
    """
    Modular DICOM loader and series organizer.
    """

    # Standard DICOM Tags
    TAG_INSTANCE_NUMBER = (0x0020, 0x0013)
    TAG_IMAGE_POSITION = (0x0020, 0x0032)
    TAG_IMAGE_ORIENTATION = (0x0020, 0x0037)
    TAG_SLICE_THICKNESS = (0x0018, 0x0050)
    TAG_ROWS = (0x0028, 0x0010)
    TAG_COLUMNS = (0x0028, 0x0011)
    TAG_PIXEL_SPACING = (0x0028, 0x0030)
    TAG_BITS_ALLOCATED = (0x0028, 0x0100)
    TAG_PIXEL_REPRESENTATION = (0x0028, 0x0103)
    TAG_RESCALE_INTERCEPT = (0x0028, 0x1052)
    TAG_RESCALE_SLOPE = (0x0028, 0x1053)
    TAG_PIXEL_DATA = (0x7FE0, 0x0010)

    @classmethod
    def load_file(cls, filepath: str) -> DICOMSlice:
        """
        Load and parse a single DICOM Part 10 file.
        """
        if not os.path.exists(filepath):
            raise FileNotFoundError(f"DICOM file not found: {filepath}")

        if HAS_PYDICOM:
            try:
                return cls._load_with_pydicom(filepath)
            except Exception:
                pass  # Fall back to native parser

        return cls._load_native(filepath)

    @classmethod
    def _load_with_pydicom(cls, filepath: str) -> DICOMSlice:
        """Parse DICOM slice using pydicom library."""
        ds = pydicom.dcmread(filepath, force=True)

        rows = int(ds.Rows)
        cols = int(ds.Columns)
        inst_num = int(getattr(ds, "InstanceNumber", 1))

        spacing = getattr(ds, "PixelSpacing", [1.0, 1.0])
        spacing_y = float(spacing[0])
        spacing_x = float(spacing[1])

        thickness = float(getattr(ds, "SliceThickness", 1.0))

        pos = getattr(ds, "ImagePositionPatient", [0.0, 0.0, 0.0])
        pos_tuple = (float(pos[0]), float(pos[1]), float(pos[2]))

        orient = getattr(ds, "ImageOrientationPatient", [1.0, 0.0, 0.0, 0.0, 1.0, 0.0])
        orient_tuple = tuple(float(v) for v in orient)

        slope = float(getattr(ds, "RescaleSlope", 1.0))
        intercept = float(getattr(ds, "RescaleIntercept", 0.0))

        raw_pixels = ds.pixel_array.astype(np.float32)
        hu_pixels = raw_pixels * slope + intercept

        # Normal vector n = row_dir x col_dir
        r_row = np.array(orient_tuple[:3], dtype=np.float64)
        r_col = np.array(orient_tuple[3:], dtype=np.float64)
        normal = np.cross(r_row, r_col)
        proj = float(np.dot(np.array(pos_tuple, dtype=np.float64), normal))

        return DICOMSlice(
            filepath=filepath,
            instance_number=inst_num,
            rows=rows,
            columns=cols,
            pixel_spacing=(spacing_y, spacing_x),
            slice_thickness=thickness,
            image_position=pos_tuple,
            image_orientation=orient_tuple,
            rescale_slope=slope,
            rescale_intercept=intercept,
            pixel_array=raw_pixels,
            hu_array=hu_pixels,
            slice_location_projection=proj,
        )

    @classmethod
    def _load_native(cls, filepath: str) -> DICOMSlice:
        """
        Native DICOM Part 10 file parser without third-party dependencies.
        Parses uncompressed Explicit and Implicit VR Little Endian formats.
        """
        with open(filepath, "rb") as f:
            data = f.read()

        if len(data) < 132:
            raise ValueError(f"File too small to be DICOM Part 10: {filepath}")

        # Check preamble and magic 'DICM'
        if data[128:132] != b"DICM":
            raise ValueError(f"Invalid DICOM Part 10 magic bytes: {filepath}")

        offset = 132
        data_len = len(data)

        # Extracted tag values
        inst_num = 1
        rows = 0
        cols = 0
        pixel_spacing = (1.0, 1.0)
        slice_thickness = 1.0
        pos = (0.0, 0.0, 0.0)
        orient = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0)
        slope = 1.0
        intercept = 0.0
        pixel_bytes = None
        pixel_rep = 0

        while offset + 8 <= data_len:
            group, elem = struct.unpack_from("<HH", data, offset)
            offset += 4

            vr_raw = data[offset:offset + 2]
            # Check if explicit VR ASCII letters
            if vr_raw.isalpha() and vr_raw.isupper():
                vr = vr_raw.decode("ascii", errors="ignore")
                offset += 2
                if vr in ("OB", "OW", "OF", "SQ", "UT", "UN"):
                    offset += 2  # Reserved 2 bytes
                    length = struct.unpack_from("<I", data, offset)[0]
                    offset += 4
                else:
                    length = struct.unpack_from("<H", data, offset)[0]
                    offset += 2
            else:
                # Implicit VR (4-byte length)
                length = struct.unpack_from("<I", data, offset - 2)[0]
                offset += 2
                vr = "UN"

            if length == 0xFFFFFFFF:
                # Undefined length sequence
                break

            if offset + length > data_len:
                length = data_len - offset

            val_bytes = data[offset:offset + length]
            offset += length

            tag = (group, elem)

            if tag == cls.TAG_ROWS:
                rows = struct.unpack("<H", val_bytes[:2])[0]
            elif tag == cls.TAG_COLUMNS:
                cols = struct.unpack("<H", val_bytes[:2])[0]
            elif tag == cls.TAG_INSTANCE_NUMBER:
                try:
                    inst_num = int(val_bytes.decode("ascii", errors="ignore").strip("\x00 ").strip())
                except ValueError:
                    pass
            elif tag == cls.TAG_PIXEL_SPACING:
                try:
                    s_str = val_bytes.decode("ascii", errors="ignore").strip("\x00 ").split("\\")
                    pixel_spacing = (float(s_str[0]), float(s_str[1]))
                except (ValueError, IndexError):
                    pass
            elif tag == cls.TAG_SLICE_THICKNESS:
                try:
                    slice_thickness = float(val_bytes.decode("ascii", errors="ignore").strip("\x00 ").strip())
                except ValueError:
                    pass
            elif tag == cls.TAG_IMAGE_POSITION:
                try:
                    p_str = val_bytes.decode("ascii", errors="ignore").strip("\x00 ").split("\\")
                    pos = (float(p_str[0]), float(p_str[1]), float(p_str[2]))
                except (ValueError, IndexError):
                    pass
            elif tag == cls.TAG_IMAGE_ORIENTATION:
                try:
                    o_str = val_bytes.decode("ascii", errors="ignore").strip("\x00 ").split("\\")
                    orient = tuple(float(v) for v in o_str[:6])
                except (ValueError, IndexError):
                    pass
            elif tag == cls.TAG_RESCALE_INTERCEPT:
                try:
                    intercept = float(val_bytes.decode("ascii", errors="ignore").strip("\x00 ").strip())
                except ValueError:
                    pass
            elif tag == cls.TAG_RESCALE_SLOPE:
                try:
                    slope = float(val_bytes.decode("ascii", errors="ignore").strip("\x00 ").strip())
                except ValueError:
                    pass
            elif tag == cls.TAG_BITS_ALLOCATED:
                pass
            elif tag == cls.TAG_PIXEL_REPRESENTATION:
                pixel_rep = struct.unpack("<H", val_bytes[:2])[0]
            elif tag == cls.TAG_PIXEL_DATA:
                pixel_bytes = val_bytes
                break

        if rows == 0 or cols == 0 or pixel_bytes is None:
            raise ValueError(f"DICOM file missing essential image data: {filepath}")

        # Unpack raw pixel buffer
        dtype = np.int16 if pixel_rep == 1 else np.uint16
        num_pixels = rows * cols
        raw_arr = np.frombuffer(pixel_bytes[:num_pixels * 2], dtype=dtype)
        if raw_arr.size != num_pixels:
            raise ValueError(f"Pixel array size mismatch: expected {num_pixels}, got {raw_arr.size}")

        raw_2d = raw_arr.reshape((rows, cols)).astype(np.float32)
        hu_2d = raw_2d * slope + intercept

        # Calculate normal projection
        r_row = np.array(orient[:3], dtype=np.float64)
        r_col = np.array(orient[3:], dtype=np.float64)
        normal = np.cross(r_row, r_col)
        proj = float(np.dot(np.array(pos, dtype=np.float64), normal))

        return DICOMSlice(
            filepath=filepath,
            instance_number=inst_num,
            rows=rows,
            columns=cols,
            pixel_spacing=pixel_spacing,
            slice_thickness=slice_thickness,
            image_position=pos,
            image_orientation=orient,
            rescale_slope=slope,
            rescale_intercept=intercept,
            pixel_array=raw_2d,
            hu_array=hu_2d,
            slice_location_projection=proj,
        )

    @classmethod
    def load_series(cls, directory: str) -> List[DICOMSlice]:
        """
        Load all valid DICOM slices in a directory and sort them physically along the scan normal.
        """
        if not os.path.isdir(directory):
            raise NotADirectoryError(f"Directory not found: {directory}")

        slices: List[DICOMSlice] = []
        for fname in os.listdir(directory):
            fpath = os.path.join(directory, fname)
            if os.path.isfile(fpath):
                try:
                    sl = cls.load_file(fpath)
                    slices.append(sl)
                except Exception:
                    # Non-DICOM files in folder are skipped safely
                    continue

        if not slices:
            raise ValueError(f"No readable DICOM slices found in {directory}")

        # Verify geometric consistency
        ref = slices[0]
        for s in slices[1:]:
            if s.rows != ref.rows or s.columns != ref.columns:
                raise ValueError(f"Inconsistent slice dimensions: {s.rows}x{s.columns} vs {ref.rows}x{ref.columns}")
            if not np.allclose(s.image_orientation, ref.image_orientation, atol=1e-3):
                raise ValueError("Inconsistent image orientation within DICOM series")

        # Sort physically along the scan slice normal projection
        slices.sort(key=lambda s: s.slice_location_projection)
        return slices

    @classmethod
    def generate_synthetic_series(
        cls,
        output_dir: str,
        num_slices: int = 16,
        rows: int = 64,
        cols: int = 64,
        pixel_spacing: Tuple[float, float] = (1.5, 1.5),
        slice_thickness: float = 2.5,
        target_center_mm: Tuple[float, float, float] = (0.0, 10.0, 20.0),
        target_radius_mm: float = 8.0,
    ) -> List[str]:
        """
        Generate a deterministic synthetic DICOM CT series representing a cylindrical anatomical phantom
        with soft tissue, cortical bone shell, and an embedded high-density surgical target lesion.

        :param output_dir: Directory where .dcm files will be saved.
        :param num_slices: Number of axial slices.
        :param rows: Image rows.
        :param cols: Image columns.
        :param pixel_spacing: (dy, dx) in mm.
        :param slice_thickness: dz in mm.
        :param target_center_mm: (x, y, z) location of surgical target in patient physical coords.
        :param target_radius_mm: Radius of surgical target in mm.
        :return: List of written file paths.
        """
        os.makedirs(output_dir, exist_ok=True)
        file_paths = []

        dy, dx = pixel_spacing
        dz = slice_thickness
        origin_x = -((cols - 1) * dx) / 2.0
        origin_y = -((rows - 1) * dy) / 2.0
        origin_z = 0.0

        target_cx, target_cy, target_cz = target_center_mm

        # Standard CT orientation (axial): row along +X, col along +Y, normal along +Z
        orient = (1.0, 0.0, 0.0, 0.0, 1.0, 0.0)

        field_radius = min(rows * dy, cols * dx) * 0.5
        phantom_radius = field_radius * 0.70
        bone_inner = phantom_radius * 0.85

        for k in range(num_slices):
            z_pos = origin_z + k * dz
            pos = (origin_x, origin_y, z_pos)

            # Construct 2D CT array in Hounsfield Units (HU)
            # Air = -1000 HU
            hu_slice = np.full((rows, cols), -1000.0, dtype=np.float32)

            for r in range(rows):
                y_mm = origin_y + r * dy
                for c in range(cols):
                    x_mm = origin_x + c * dx

                    # Cylindrical phantom body
                    dist_axis = np.hypot(x_mm, y_mm)
                    if dist_axis < phantom_radius:
                        # Soft tissue: ~40 HU
                        hu_slice[r, c] = 40.0

                        # Cortical bone boundary shell: ~800 HU
                        if bone_inner <= dist_axis < phantom_radius:
                            hu_slice[r, c] = 800.0

                    # Embedded target spherical lesion (~280 HU)
                    dist_3d = np.sqrt((x_mm - target_cx) ** 2 + (y_mm - target_cy) ** 2 + (z_pos - target_cz) ** 2)
                    if dist_3d <= target_radius_mm:
                        hu_slice[r, c] = 280.0

            # Convert HU to stored pixel value: raw = (HU - intercept) / slope
            # Standard CT: slope = 1.0, intercept = -1024.0 -> raw = HU + 1024
            raw_pixels = np.clip(hu_slice + 1024.0, 0, 4095).astype(np.uint16)

            # Encode into DICOM Part 10 binary stream
            fpath = os.path.join(output_dir, f"slice_{k + 1:03d}.dcm")
            cls._write_part10_file(
                filepath=fpath,
                instance_number=k + 1,
                rows=rows,
                cols=cols,
                pixel_spacing=pixel_spacing,
                slice_thickness=slice_thickness,
                pos=pos,
                orient=orient,
                slope=1.0,
                intercept=-1024.0,
                pixel_array=raw_pixels,
            )
            file_paths.append(fpath)

        return file_paths

    @classmethod
    def _write_part10_file(
        cls,
        filepath: str,
        instance_number: int,
        rows: int,
        cols: int,
        pixel_spacing: Tuple[float, float],
        slice_thickness: float,
        pos: Tuple[float, float, float],
        orient: Tuple[float, float, float, float, float, float],
        slope: float,
        intercept: float,
        pixel_array: np.ndarray,
    ):
        """Write single DICOM Part 10 uncompressed file."""
        def encode_elem(grp: int, elm: int, vr_str: str, val_bytes: bytes) -> bytes:
            tag = struct.pack("<HH", grp, elm)
            vr_b = vr_str.encode("ascii")
            if vr_str in ("OB", "OW", "OF", "SQ", "UT", "UN"):
                # 4-byte length with 2 reserved bytes
                return tag + vr_b + b"\x00\x00" + struct.pack("<I", len(val_bytes)) + val_bytes
            else:
                # 2-byte length
                # Pad value to even length if needed
                if len(val_bytes) % 2 != 0:
                    val_bytes += b" "
                return tag + vr_b + struct.pack("<H", len(val_bytes)) + val_bytes

        stream = bytearray()
        # 128-byte preamble + 'DICM'
        stream.extend(b"\x00" * 128)
        stream.extend(b"DICM")

        # Basic identification and geometry elements
        stream.extend(encode_elem(0x0020, 0x0013, "IS", str(instance_number).encode("ascii")))
        stream.extend(encode_elem(0x0018, 0x0050, "DS", f"{slice_thickness:.3f}".encode("ascii")))
        stream.extend(encode_elem(0x0028, 0x0010, "US", struct.pack("<H", rows)))
        stream.extend(encode_elem(0x0028, 0x0011, "US", struct.pack("<H", cols)))
        stream.extend(encode_elem(0x0028, 0x0030, "DS", f"{pixel_spacing[0]:.4f}\\{pixel_spacing[1]:.4f}".encode("ascii")))
        stream.extend(encode_elem(0x0020, 0x0032, "DS", f"{pos[0]:.3f}\\{pos[1]:.3f}\\{pos[2]:.3f}".encode("ascii")))
        stream.extend(encode_elem(
            0x0020, 0x0037, "DS",
            f"{orient[0]:.4f}\\{orient[1]:.4f}\\{orient[2]:.4f}\\{orient[3]:.4f}\\{orient[4]:.4f}\\{orient[5]:.4f}".encode("ascii")
        ))
        stream.extend(encode_elem(0x0028, 0x0100, "US", struct.pack("<H", 16)))
        stream.extend(encode_elem(0x0028, 0x0101, "US", struct.pack("<H", 12)))
        stream.extend(encode_elem(0x0028, 0x0102, "US", struct.pack("<H", 11)))
        stream.extend(encode_elem(0x0028, 0x0103, "US", struct.pack("<H", 0)))  # Unsigned
        stream.extend(encode_elem(0x0028, 0x1052, "DS", f"{intercept:.1f}".encode("ascii")))
        stream.extend(encode_elem(0x0028, 0x1053, "DS", f"{slope:.1f}".encode("ascii")))

        # Pixel data element (OW)
        raw_bytes = pixel_array.tobytes()
        stream.extend(encode_elem(0x7FE0, 0x0010, "OW", raw_bytes))

        with open(filepath, "wb") as f:
            f.write(stream)
