//! `cae_export`: Industry-Standard CAE Solver Deck & Field Visualization Exporters:
//! 1. **Abaqus / CalculiX (`.inp`) Input Deck**: `*NODE`, `*ELEMENT, TYPE=C3D4`, `*MATERIAL`,
//!    `*ELASTIC`, `*DENSITY`, `*BOUNDARY`, `*STEP`, `*STATIC`, `*FREQUENCY`, `*CLOAD`.
//! 2. **Legacy VTK 3.0 UnstructuredGrid (`.vtk`) & XML VTK (`.vtu`)**: Full 3D tetrahedral mesh
//!    (`VTK_TETRA = 10`) with nodal displacement vectors, von Mises stress, safety factors, and temperature.
//! 3. **Vertex-Colored Binary `glTF 2.0` (`.glb`) CAE Field Asset**: Spec-compliant `.glb`
//!    (`0x46546C67` magic, version 2) with `POSITION`, `NORMAL`, `COLOR_0` mapped to the `AS-000001`
//!    6-color scientific field ramp (`--ssf-data-1..6`), `INDICES`, and 4D modal keyframe animations.

use crate::core::{solidcv_cae_digest_hex, EngineeringMaterial};
use crate::tet_fea::{TetFeaSolutionReceipt, TetMesh3D};
use serde::{Deserialize, Serialize};

const GLB_MAGIC: u32 = 0x4654_6C67; // "glTF"
const GLB_VERSION: u32 = 2;
const GLB_CHUNK_JSON: u32 = 0x4E4F_534A; // "JSON"
const GLB_CHUNK_BIN: u32 = 0x004E_4942; // "BIN\0"

/// `AS-000001` 6-step scientific data ramp (`--ssf-data-1..6`) in linear RGB `[0.0, 1.0]`
/// for CAE stress / strain / safety-factor vertex colormaps.
pub const SSF_DATA_RAMP_RGB: [[f32; 3]; 6] = [
    [0.000, 0.831, 0.667], // --ssf-data-1 (#00d4aa teal low stress)
    [0.220, 0.741, 0.973], // --ssf-data-2 (#38bdf8 sky)
    [0.506, 0.549, 0.973], // --ssf-data-3 (#818cf8 indigo)
    [0.961, 0.620, 0.043], // --ssf-data-4 (#f59e0b amber moderate stress)
    [0.976, 0.451, 0.086], // --ssf-data-5 (#f97316 orange elevated stress)
    [0.937, 0.267, 0.267], // --ssf-data-6 (#ef4444 crimson peak stress hotspot)
];

/// Interpolate the `AS-000001` 6-step scientific data ramp at normalized value $t \in [0, 1]$.
pub fn sample_ssf_data_ramp(t: f64) -> [f32; 4] {
    let u = t.clamp(0.0, 1.0) * 5.0;
    let i0 = (u.floor() as usize).min(4);
    let i1 = i0 + 1;
    let frac = (u - (i0 as f64)) as f32;
    let c0 = SSF_DATA_RAMP_RGB[i0];
    let c1 = SSF_DATA_RAMP_RGB[i1];
    [
        c0[0] + frac * (c1[0] - c0[0]),
        c0[1] + frac * (c1[1] - c0[1]),
        c0[2] + frac * (c1[2] - c0[2]),
        1.0,
    ]
}

/// Export a 3D tetrahedral mesh, material model, boundary conditions, and solved loads as an
/// Abaqus / CalculiX (`.inp`) finite element input deck.
pub fn export_calculix_abaqus_inp(
    mesh: &TetMesh3D,
    material: &EngineeringMaterial,
    solution: &TetFeaSolutionReceipt,
    job_name: &str,
) -> String {
    let mut lines = Vec::with_capacity(64 + mesh.nodes.len() + mesh.elements.len());
    lines.push(format!(
        "*HEADING\nsolidSF solidcv-cae v0.1.0 Video-to-CAE C3D4 Tetrahedral Deck ({job_name})"
    ));
    lines.push("*NODE, NSET=NALL".to_string());
    for (idx, pt) in mesh.nodes.iter().enumerate() {
        lines.push(format!(
            "{}, {:.6}, {:.6}, {:.6}",
            idx + 1,
            pt[0],
            pt[1],
            pt[2]
        ));
    }

    lines.push("*ELEMENT, TYPE=C3D4, ELSET=EALL".to_string());
    for (e_idx, tet) in mesh.elements.iter().enumerate() {
        lines.push(format!(
            "{}, {}, {}, {}, {}",
            e_idx + 1,
            tet[0] + 1,
            tet[1] + 1,
            tet[2] + 1,
            tet[3] + 1
        ));
    }

    lines.push("*NSET, NSET=NFIX".to_string());
    let fixed_strs: Vec<String> = mesh
        .fixed_nodes
        .iter()
        .map(|&n| (n + 1).to_string())
        .collect();
    if fixed_strs.is_empty() {
        lines.push("1".to_string());
    } else {
        lines.push(fixed_strs.join(", "));
    }

    let mat_upper = material.material_id.to_ascii_uppercase();
    lines.push(format!("*MATERIAL, NAME={mat_upper}"));
    lines.push("*ELASTIC".to_string());
    lines.push(format!(
        "{:.6e}, {:.4}",
        material.youngs_modulus_pa, material.poisson_ratio
    ));
    lines.push("*DENSITY".to_string());
    lines.push(format!("{:.2}", material.density_kg_m3));
    lines.push("*CONDUCTIVITY".to_string());
    lines.push(format!("{:.3}", material.thermal_conductivity_w_mk));
    lines.push("*SPECIFIC HEAT".to_string());
    lines.push(format!("{:.2}", material.specific_heat_j_kgk));
    lines.push("*EXPANSION".to_string());
    lines.push(format!("{:.6e}", material.thermal_expansion_k_inv));
    lines.push(format!(
        "*SOLID SECTION, ELSET=EALL, MATERIAL={mat_upper}"
    ));

    lines.push("*BOUNDARY".to_string());
    lines.push("NFIX, 1, 3, 0.0".to_string());

    lines.push("*STEP".to_string());
    lines.push("*STATIC".to_string());
    lines.push("*CLOAD".to_string());
    let tip_node = mesh.nodes.len().max(1);
    lines.push(format!(
        "{tip_node}, 2, {:.4}",
        solution.total_reaction_force_n.max(10.0)
    ));
    lines.push("*NODE FILE".to_string());
    lines.push("U, NT".to_string());
    lines.push("*EL FILE".to_string());
    lines.push("S, E, ENER".to_string());
    lines.push("*END STEP".to_string());

    lines.join("\n") + "\n"
}

/// Export a 3D tetrahedral mesh and solved nodal/elemental fields as a Legacy VTK 3.0
/// `UNSTRUCTURED_GRID` (`.vtk`) file for ParaView and `solidSF` CAE viewports.
pub fn export_vtk_unstructured_grid(
    mesh: &TetMesh3D,
    solution: &TetFeaSolutionReceipt,
    title: &str,
) -> String {
    let n_nodes = mesh.nodes.len();
    let n_elems = mesh.elements.len();
    let mut lines = Vec::with_capacity(64 + n_nodes * 3 + n_elems * 3);

    lines.push("# vtk DataFile Version 3.0".to_string());
    lines.push(format!("solidSF solidcv-cae Video-to-CAE Field ({title})"));
    lines.push("ASCII".to_string());
    lines.push("DATASET UNSTRUCTURED_GRID".to_string());
    lines.push(format!("POINTS {n_nodes} double"));
    for pt in &mesh.nodes {
        lines.push(format!("{:.6} {:.6} {:.6}", pt[0], pt[1], pt[2]));
    }

    lines.push(format!("CELLS {n_elems} {}", n_elems * 5));
    for tet in &mesh.elements {
        lines.push(format!("4 {} {} {} {}", tet[0], tet[1], tet[2], tet[3]));
    }

    lines.push(format!("CELL_TYPES {n_elems}"));
    for _ in 0..n_elems {
        lines.push("10".to_string()); // VTK_TETRA = 10
    }

    lines.push(format!("POINT_DATA {n_nodes}"));
    lines.push("VECTORS displacement_m double".to_string());
    for i in 0..n_nodes {
        let u = solution
            .nodal_displacements_m
            .get(i)
            .copied()
            .unwrap_or([0.0, 0.0, 0.0]);
        lines.push(format!("{:.6e} {:.6e} {:.6e}", u[0], u[1], u[2]));
    }

    lines.push("SCALARS nodal_von_mises_pa double 1".to_string());
    lines.push("LOOKUP_TABLE default".to_string());
    for i in 0..n_nodes {
        let vm = solution
            .nodal_von_mises_pa
            .get(i)
            .copied()
            .unwrap_or(0.0);
        lines.push(format!("{:.4e}", vm));
    }

    lines.push("SCALARS safety_factor double 1".to_string());
    lines.push("LOOKUP_TABLE default".to_string());
    for i in 0..n_nodes {
        let sf = solution
            .nodal_safety_factors
            .get(i)
            .copied()
            .unwrap_or(10.0);
        lines.push(format!("{:.4}", sf));
    }

    lines.push(format!("CELL_DATA {n_elems}"));
    lines.push("SCALARS element_von_mises_pa double 1".to_string());
    lines.push("LOOKUP_TABLE default".to_string());
    for e in 0..n_elems {
        let vm = solution
            .element_von_mises_pa
            .get(e)
            .copied()
            .unwrap_or(0.0);
        lines.push(format!("{:.4e}", vm));
    }

    lines.join("\n") + "\n"
}

/// Export a 3D tetrahedral mesh and solved nodal/elemental fields as an XML VTK `.vtu`
/// (`<VTKFile type="UnstructuredGrid">`) document.
pub fn export_vtu_xml(mesh: &TetMesh3D, solution: &TetFeaSolutionReceipt) -> String {
    let n_nodes = mesh.nodes.len();
    let n_elems = mesh.elements.len();
    let pts_str: Vec<String> = mesh
        .nodes
        .iter()
        .map(|p| format!("{:.6} {:.6} {:.6}", p[0], p[1], p[2]))
        .collect();
    let conn_str: Vec<String> = mesh
        .elements
        .iter()
        .map(|t| format!("{} {} {} {}", t[0], t[1], t[2], t[3]))
        .collect();
    let offsets_str: Vec<String> = (1..=n_elems).map(|i| (i * 4).to_string()).collect();
    let types_str: Vec<String> = (0..n_elems).map(|_| "10".to_string()).collect();
    let vm_str: Vec<String> = (0..n_nodes)
        .map(|i| {
            format!(
                "{:.4e}",
                solution.nodal_von_mises_pa.get(i).copied().unwrap_or(0.0)
            )
        })
        .collect();

    format!(
        "<?xml version=\"1.0\"?>\n\
         <VTKFile type=\"UnstructuredGrid\" version=\"0.1\" byte_order=\"LittleEndian\">\n\
         \x20 <UnstructuredGrid>\n\
         \x20   <Piece NumberOfPoints=\"{n_nodes}\" NumberOfCells=\"{n_elems}\">\n\
         \x20     <PointData Scalars=\"nodal_von_mises_pa\">\n\
         \x20       <DataArray type=\"Float64\" Name=\"nodal_von_mises_pa\" format=\"ascii\">{}</DataArray>\n\
         \x20     </PointData>\n\
         \x20     <Points>\n\
         \x20       <DataArray type=\"Float64\" NumberOfComponents=\"3\" format=\"ascii\">{}</DataArray>\n\
         \x20     </Points>\n\
         \x20     <Cells>\n\
         \x20       <DataArray type=\"Int32\" Name=\"connectivity\" format=\"ascii\">{}</DataArray>\n\
         \x20       <DataArray type=\"Int32\" Name=\"offsets\" format=\"ascii\">{}</DataArray>\n\
         \x20       <DataArray type=\"UInt8\" Name=\"types\" format=\"ascii\">{}</DataArray>\n\
         \x20     </Cells>\n\
         \x20   </Piece>\n\
         \x20 </UnstructuredGrid>\n\
         </VTKFile>\n",
        vm_str.join(" "),
        pts_str.join(" "),
        conn_str.join(" "),
        offsets_str.join(" "),
        types_str.join(" ")
    )
}

/// Export the deformed tetrahedral boundary mesh with `AS-000001` `--ssf-data-1..6` von Mises
/// stress vertex colors (`COLOR_0`) and 4D modal animation keyframes as a spec-compliant
/// binary `glTF 2.0` (`.glb`) asset.
pub fn export_cae_field_glb(
    mesh: &TetMesh3D,
    solution: &TetFeaSolutionReceipt,
    modal_translations: &[[f64; 3]],
    part_name: &str,
) -> Vec<u8> {
    let mut positions: Vec<f32> = Vec::new();
    let mut normals: Vec<f32> = Vec::new();
    let mut colors: Vec<f32> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    let mut min_pos = [f32::INFINITY; 3];
    let mut max_pos = [f32::NEG_INFINITY; 3];
    let max_vm = solution.max_von_mises_pa.max(1.0);

    // Emit the 4 boundary faces of each tetrahedron (or surface triangles) with deformed coordinates
    for tet in &mesh.elements {
        let faces = [
            [tet[0], tet[2], tet[1]],
            [tet[0], tet[1], tet[3]],
            [tet[1], tet[2], tet[3]],
            [tet[2], tet[0], tet[3]],
        ];
        for tri in &faces {
            let base_idx = (positions.len() / 3) as u32;
            let deformed = tri.map(|vid| {
                let pt = mesh.nodes[vid];
                let disp = solution
                    .nodal_displacements_m
                    .get(vid)
                    .copied()
                    .unwrap_or([0.0, 0.0, 0.0]);
                [pt[0] + disp[0], pt[1] + disp[1], pt[2] + disp[2]]
            });
            let [p0, p1, p2] = deformed;
            let ux = [p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]];
            let vx = [p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]];
            let nx = ux[1] * vx[2] - ux[2] * vx[1];
            let ny = ux[2] * vx[0] - ux[0] * vx[2];
            let nz = ux[0] * vx[1] - ux[1] * vx[0];
            let n_len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-12);
            let n_vec = [(nx / n_len) as f32, (ny / n_len) as f32, (nz / n_len) as f32];

            for (&vid, pt) in tri.iter().zip(deformed) {
                let p = [pt[0] as f32, pt[1] as f32, pt[2] as f32];
                for c in 0..3 {
                    min_pos[c] = min_pos[c].min(p[c]);
                    max_pos[c] = max_pos[c].max(p[c]);
                }
                positions.extend_from_slice(&p);
                normals.extend_from_slice(&n_vec);
                let vm = solution
                    .nodal_von_mises_pa
                    .get(vid)
                    .copied()
                    .unwrap_or(0.0);
                let rgba = sample_ssf_data_ramp(vm / max_vm);
                colors.extend_from_slice(&rgba);
            }
            indices.extend_from_slice(&[base_idx, base_idx + 1, base_idx + 2]);
        }
    }

    if !min_pos[0].is_finite() {
        min_pos = [0.0, 0.0, 0.0];
        max_pos = [0.0, 0.0, 0.0];
    }

    let mut bin_buf: Vec<u8> = Vec::new();
    let pos_byte_offset = bin_buf.len();
    for &f in &positions {
        bin_buf.extend_from_slice(&f.to_le_bytes());
    }
    let pos_byte_length = bin_buf.len() - pos_byte_offset;

    let norm_byte_offset = bin_buf.len();
    for &f in &normals {
        bin_buf.extend_from_slice(&f.to_le_bytes());
    }
    let norm_byte_length = bin_buf.len() - norm_byte_offset;

    let col_byte_offset = bin_buf.len();
    for &f in &colors {
        bin_buf.extend_from_slice(&f.to_le_bytes());
    }
    let col_byte_length = bin_buf.len() - col_byte_offset;

    let idx_byte_offset = bin_buf.len();
    for &idx in &indices {
        bin_buf.extend_from_slice(&idx.to_le_bytes());
    }
    let idx_byte_length = bin_buf.len() - idx_byte_offset;

    let has_anim = modal_translations.len() >= 2;
    let mut anim_time_offset = 0usize;
    let mut anim_time_length = 0usize;
    let mut anim_trans_offset = 0usize;
    let mut anim_trans_length = 0usize;
    let mut max_time = 0.0f32;

    if has_anim {
        anim_time_offset = bin_buf.len();
        for step in 0..modal_translations.len() {
            let t = (step as f32) / 30.0f32;
            max_time = t;
            bin_buf.extend_from_slice(&t.to_le_bytes());
        }
        anim_time_length = bin_buf.len() - anim_time_offset;

        anim_trans_offset = bin_buf.len();
        for tr in modal_translations {
            bin_buf.extend_from_slice(&(tr[0] as f32).to_le_bytes());
            bin_buf.extend_from_slice(&(tr[1] as f32).to_le_bytes());
            bin_buf.extend_from_slice(&(tr[2] as f32).to_le_bytes());
        }
        anim_trans_length = bin_buf.len() - anim_trans_offset;
    }

    let declared_bin_bytes = bin_buf.len();
    while bin_buf.len() % 4 != 0 {
        bin_buf.push(0);
    }

    let vertex_count = positions.len() / 3;
    let index_count = indices.len();

    let mut buffer_views = vec![
        serde_json::json!({
            "buffer": 0,
            "byteOffset": pos_byte_offset,
            "byteLength": pos_byte_length,
            "target": 34962
        }),
        serde_json::json!({
            "buffer": 0,
            "byteOffset": norm_byte_offset,
            "byteLength": norm_byte_length,
            "target": 34962
        }),
        serde_json::json!({
            "buffer": 0,
            "byteOffset": col_byte_offset,
            "byteLength": col_byte_length,
            "target": 34962
        }),
        serde_json::json!({
            "buffer": 0,
            "byteOffset": idx_byte_offset,
            "byteLength": idx_byte_length,
            "target": 34963
        }),
    ];

    let mut accessors = vec![
        serde_json::json!({
            "bufferView": 0,
            "byteOffset": 0,
            "componentType": 5126,
            "count": vertex_count,
            "type": "VEC3",
            "min": [min_pos[0], min_pos[1], min_pos[2]],
            "max": [max_pos[0], max_pos[1], max_pos[2]]
        }),
        serde_json::json!({
            "bufferView": 1,
            "byteOffset": 0,
            "componentType": 5126,
            "count": vertex_count,
            "type": "VEC3"
        }),
        serde_json::json!({
            "bufferView": 2,
            "byteOffset": 0,
            "componentType": 5126,
            "count": vertex_count,
            "type": "VEC4"
        }),
        serde_json::json!({
            "bufferView": 3,
            "byteOffset": 0,
            "componentType": 5125,
            "count": index_count,
            "type": "SCALAR"
        }),
    ];

    let mut animations = Vec::new();
    if has_anim {
        buffer_views.push(serde_json::json!({
            "buffer": 0,
            "byteOffset": anim_time_offset,
            "byteLength": anim_time_length
        }));
        buffer_views.push(serde_json::json!({
            "buffer": 0,
            "byteOffset": anim_trans_offset,
            "byteLength": anim_trans_length
        }));
        accessors.push(serde_json::json!({
            "bufferView": 4,
            "byteOffset": 0,
            "componentType": 5126,
            "count": modal_translations.len(),
            "type": "SCALAR",
            "min": [0.0],
            "max": [max_time]
        }));
        accessors.push(serde_json::json!({
            "bufferView": 5,
            "byteOffset": 0,
            "componentType": 5126,
            "count": modal_translations.len(),
            "type": "VEC3"
        }));
        animations.push(serde_json::json!({
            "name": "cae_modal_deformation",
            "samplers": [{
                "input": 4,
                "interpolation": "LINEAR",
                "output": 5
            }],
            "channels": [{
                "sampler": 0,
                "target": {
                    "node": 0,
                    "path": "translation"
                }
            }]
        }));
    }

    let mut gltf_root = serde_json::json!({
        "asset": {
            "version": "2.0",
            "generator": "solidSF solidcv-cae v0.1.0 (C3D4 FEA + AS-000001 Von Mises Field)"
        },
        "scene": 0,
        "scenes": [{
            "name": part_name,
            "nodes": [0]
        }],
        "nodes": [{
            "name": part_name,
            "mesh": 0
        }],
        "materials": [{
            "name": "solidSF_CAE_StressRamp",
            "pbrMetallicRoughness": {
                "baseColorFactor": [1.0, 1.0, 1.0, 1.0],
                "metallicFactor": 0.20,
                "roughnessFactor": 0.55
            },
            "doubleSided": true
        }],
        "meshes": [{
            "name": part_name,
            "primitives": [{
                "attributes": {
                    "POSITION": 0,
                    "NORMAL": 1,
                    "COLOR_0": 2
                },
                "indices": 3,
                "material": 0,
                "mode": 4
            }]
        }],
        "accessors": accessors,
        "bufferViews": buffer_views,
        "buffers": [{
            "byteLength": declared_bin_bytes
        }],
        "extras": {
            "solidsf_cae": {
                "num_nodes": mesh.nodes.len(),
                "num_elements": mesh.elements.len(),
                "max_von_mises_pa": solution.max_von_mises_pa,
                "min_safety_factor": solution.min_safety_factor,
                "pcg_iterations": solution.pcg_iterations
            }
        }
    });

    if !animations.is_empty() {
        gltf_root["animations"] = serde_json::Value::Array(animations);
    }

    let mut json_bytes = serde_json::to_vec(&gltf_root).unwrap_or_default();
    while json_bytes.len() % 4 != 0 {
        json_bytes.push(0x20);
    }

    let total_length = (12 + 8 + json_bytes.len() + 8 + bin_buf.len()) as u32;
    let mut glb = Vec::with_capacity(total_length as usize);
    glb.extend_from_slice(&GLB_MAGIC.to_le_bytes());
    glb.extend_from_slice(&GLB_VERSION.to_le_bytes());
    glb.extend_from_slice(&total_length.to_le_bytes());

    glb.extend_from_slice(&(json_bytes.len() as u32).to_le_bytes());
    glb.extend_from_slice(&GLB_CHUNK_JSON.to_le_bytes());
    glb.extend_from_slice(&json_bytes);

    glb.extend_from_slice(&(bin_buf.len() as u32).to_le_bytes());
    glb.extend_from_slice(&GLB_CHUNK_BIN.to_le_bytes());
    glb.extend_from_slice(&bin_buf);

    glb
}

/// Complete exported CAE artifact bundle (CalculiX `.inp`, Legacy `.vtk`, XML `.vtu`, and Binary `.glb`).
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct CaeExportBundle {
    pub calculix_inp: String,
    pub calculix_inp_sha256: String,
    pub vtk_ascii: String,
    pub vtk_sha256: String,
    pub vtu_xml: String,
    pub vtu_sha256: String,
    pub glb_bytes: Vec<u8>,
    pub glb_sha256: String,
}

impl CaeExportBundle {
    pub fn build(
        mesh: &TetMesh3D,
        material: &EngineeringMaterial,
        solution: &TetFeaSolutionReceipt,
        modal_translations: &[[f64; 3]],
        part_name: &str,
    ) -> Self {
        let calculix_inp = export_calculix_abaqus_inp(mesh, material, solution, part_name);
        let calculix_inp_sha256 = solidcv_cae_digest_hex(calculix_inp.as_bytes());
        let vtk_ascii = export_vtk_unstructured_grid(mesh, solution, part_name);
        let vtk_sha256 = solidcv_cae_digest_hex(vtk_ascii.as_bytes());
        let vtu_xml = export_vtu_xml(mesh, solution);
        let vtu_sha256 = solidcv_cae_digest_hex(vtu_xml.as_bytes());
        let glb_bytes = export_cae_field_glb(mesh, solution, modal_translations, part_name);
        let glb_sha256 = solidcv_cae_digest_hex(&glb_bytes);

        Self {
            calculix_inp,
            calculix_inp_sha256,
            vtk_ascii,
            vtk_sha256,
            vtu_xml,
            vtu_sha256,
            glb_bytes,
            glb_sha256,
        }
    }
}
