# SurgiTwin — Medical Image Segmentation & Target Extraction

## 1. Overview & Limitations

The medical segmentation subsystem isolates anatomical structures or surgical targets from 3D CT volumes using deterministic thresholding, mathematical morphology, and 3D connected-component labeling.

> [!NOTE]
> **Engineering Scope**:
> Simple intensity windowing and morphological filtering are classical computer vision operations. They are implemented here to provide a transparent, reproducible simulation pipeline without external heavy AI model dependencies. They must **not** be represented as clinically validated diagnostic AI.

## 2. Segmentation Pipeline

```
Reconstructed 3D Volume (HU)
            |
            v
+-------------------------------+
| Dual-Threshold Intensity Mask |  (e.g., HU in [200, 400])
+-------------------------------+
            |
            v
+-------------------------------+
| 3D Morphological Filtering    |  (Opening removes noise, Closing fills voids)
+-------------------------------+
            |
            v
+-------------------------------+
| 3D Connected Component Analysis| (ndimage.label 6-connectivity)
+-------------------------------+
            |
            v
+-------------------------------+
| Volume Filtering & Selection   | (Select target component > min_volume_mm3)
+-------------------------------+
            |
            v
+-------------------------------+
| Anatomical Target Extractor   | (Centroid, Bounding Box, Equivalent Radius)
+-------------------------------+
```

## 3. Mathematical Centroid & Metrics Extraction

Given a binary 3D segmentation mask $M(k, j, i) \in \{0, 1\}$ containing $N = \sum M(k, j, i)$ foreground voxels:

### Discrete Voxel Centroid
$$
\bar{i} = \frac{1}{N} \sum_{M=1} i, \quad \bar{j} = \frac{1}{N} \sum_{M=1} j, \quad \bar{k} = \frac{1}{N} \sum_{M=1} k
$$

### Continuous Physical Patient Centroid
$$
\mathbf{P}_{target} = \mathbf{P}_0 + \bar{i} s_x \mathbf{r} + \bar{j} s_y \mathbf{c} + \bar{k} s_z \hat{\mathbf{n}} \quad [\text{mm}]
$$

### Physical Volume & Equivalent Radius
$$
V = N \times (s_x \cdot s_y \cdot s_z) \quad [\text{mm}^3]
$$
$$
R_{equiv} = \left(\frac{3 V}{4 \pi}\right)^{1/3} \quad [\text{mm}]
$$

## 4. 3D Surface Mesh Export (STL)

To facilitate visualization in surgical planners (such as 3D Slicer or CAD tools), the boundary voxels of the segmented mask are exported as an ASCII/binary stereolithography (STL) surface mesh stored at:
`docs/models/segmented_target.stl`.

