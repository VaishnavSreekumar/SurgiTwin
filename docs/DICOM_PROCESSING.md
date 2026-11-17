# SurgiTwin — DICOM Processing & Volume Reconstruction

## 1. Scope & Medical Imaging Disclaimer

The DICOM processing subsystem ingests standard cross-sectional medical scans (CT and MRI) in DICOM Part 10 format and reconstructs 3D volumetric patient representations.

> [!CAUTION]
> **Research & Educational Prototype Disclaimer**:
> This software is intended strictly for simulation, engineering education, and robotic digital twin research. It is **not** certified under FDA 510(k), CE mark (MDR 2017/745), or ISO 13485, and must never be utilized for primary clinical diagnosis, surgical planning on live human patients, or clinical decision support.

## 2. DICOM Part 10 Specification & Metadata Model

The pipeline extracts essential spatial and modality tags per the DICOM PS 3.3 specification:

| Attribute Name | Tag | VR | Description |
|---|---|---|---|
| `Rows` | `(0028, 0010)` | US | Number of rows in image matrix ($N_y$) |
| `Columns` | `(0028, 0011)` | US | Number of columns in image matrix ($N_x$) |
| `PixelSpacing` | `(0028, 0030)` | DS | Adjacent pixel spacing $(s_y, s_x)$ in mm |
| `SliceThickness` | `(0018, 0050)` | DS | Nominal slice acquisition thickness ($s_z$) in mm |
| `ImagePositionPatient` | `(0020, 0032)` | DS | Physical $(X, Y, Z)$ coordinates of upper-left voxel $(0, 0, 0)$ in mm |
| `ImageOrientationPatient` | `(0020, 0037)` | DS | Direction cosines of first row and column $[r_{xx}, r_{xy}, r_{xz}, c_{yx}, c_{yy}, c_{yz}]$ |
| `RescaleIntercept` | `(0028, 1052)` | DS | Intercept $b$ for Hounsfield Unit conversion |
| `RescaleSlope` | `(0028, 1053)` | DS | Slope $m$ for Hounsfield Unit conversion |

### Modality Value Conversion (Hounsfield Units)
Raw stored integer pixel values $p_{raw}$ are converted to physical CT numbers (Hounsfield Units, HU):
$$
\text{HU} = p_{raw} \times \text{RescaleSlope} + \text{RescaleIntercept}
$$
Standard CT calibration benchmarks:
- Air: $-1000$ HU
- Lung parenchyma: $-900$ to $-400$ HU
- Water: $0$ HU
- Soft tissue (muscle/organ): $+20$ to $+80$ HU
- Cortical bone: $+400$ to $+1500$ HU

## 3. Physical Slice Normal Sorting

Relying on filename sorting (e.g. `slice_01.dcm`) causes spatial artifacts if files are renamed or interleaved. SurgiTwin computes the true acquisition scan normal $\mathbf{n}$ from direction cosines:

$$
\mathbf{r} = \begin{bmatrix} r_{xx} \\ r_{xy} \\ r_{xz} \end{bmatrix}, \quad \mathbf{c} = \begin{bmatrix} c_{yx} \\ c_{yy} \\ c_{yz} \end{bmatrix}, \quad \hat{\mathbf{n}} = \frac{\mathbf{r} \times \mathbf{c}}{\|\mathbf{r} \times \mathbf{c}\|}
$$

Each slice position $\mathbf{P}_k$ is projected onto $\hat{\mathbf{n}}$:
$$
s_k = \mathbf{P}_k \cdot \hat{\mathbf{n}}
$$
Slices are ordered strictly along $s_k$, guaranteeing physically monotonic slice stacking.

## 4. Affine Voxel-to-Patient Coordinate Mapping

The DICOM Patient Coordinate System adheres to the **LPS convention** (Left, Posterior, Superior):
- $+X$: Directed towards patient Left
- $+Y$: Directed towards patient Posterior
- $+Z$: Directed towards patient Superior (cranial)

The mapping between discrete voxel grid coordinates $(i, j, k)$ and continuous physical coordinates $(X_p, Y_p, Z_p)$ in millimeters is expressed as an affine transformation:

$$
\begin{bmatrix} X_p \\ Y_p \\ Z_p \\ 1 \end{bmatrix} =
\begin{bmatrix}
s_x r_{xx} & s_y c_{yx} & s_z n_x & X_0 \\
s_x r_{xy} & s_y c_{yy} & s_z n_y & Y_0 \\
s_x r_{xz} & s_y c_{yz} & s_z n_z & Z_0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\begin{bmatrix} i \\ j \\ k \\ 1 \end{bmatrix}
$$

where $(X_0, Y_0, Z_0)$ is the physical position of voxel $(0, 0, 0)$ on the reference slice.

