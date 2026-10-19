use solidcv_cae::cae_export::export_cae_field_glb;
use solidcv_cae::tet_fea::{TetFeaSolutionReceipt, TetMesh3D};

fn field_glb(displacements: Vec<[f64; 3]>) -> Vec<u8> {
    let mesh = TetMesh3D::from_nodes_and_tets(
        vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
        ],
        vec![[0, 1, 2, 3]],
        vec![],
        vec![],
    )
    .unwrap();
    let solution = TetFeaSolutionReceipt {
        num_nodes: 4,
        num_elements: 1,
        num_dofs: 12,
        pcg_iterations: 0,
        pcg_residual_norm: 0.0,
        nodal_displacements_m: displacements,
        nodal_von_mises_pa: vec![0.0; 4],
        nodal_safety_factors: vec![1.0; 4],
        element_strains: vec![],
        element_stresses: vec![],
        element_von_mises_pa: vec![0.0],
        max_displacement_m: 0.0,
        max_von_mises_pa: 0.0,
        min_safety_factor: 1.0,
        total_strain_energy_j: 0.0,
        total_reaction_force_n: 0.0,
    };
    export_cae_field_glb(&mesh, &solution, &[], "deformed tetrahedron")
}

fn attribute(glb: &[u8], name: &str) -> Vec<[f32; 3]> {
    let json_len = u32::from_le_bytes(glb[12..16].try_into().unwrap()) as usize;
    let root: serde_json::Value = serde_json::from_slice(&glb[20..20 + json_len]).unwrap();
    let index = root["meshes"][0]["primitives"][0]["attributes"][name]
        .as_u64()
        .unwrap() as usize;
    let accessor = &root["accessors"][index];
    assert_eq!(accessor["type"], "VEC3");
    assert_eq!(accessor["componentType"], 5126);
    let view = &root["bufferViews"][accessor["bufferView"].as_u64().unwrap() as usize];
    let start = 28 + json_len + view["byteOffset"].as_u64().unwrap() as usize;
    let len = accessor["count"].as_u64().unwrap() as usize * 12;
    glb[start..start + len]
        .chunks_exact(12)
        .map(|point| {
            std::array::from_fn(|axis| {
                f32::from_le_bytes(point[axis * 4..axis * 4 + 4].try_into().unwrap())
            })
        })
        .collect()
}

fn assert_normal(actual: [f32; 3], expected: [f32; 3]) {
    for axis in 0..3 {
        assert!(
            (actual[axis] - expected[axis]).abs() < 1e-6,
            "{actual:?} != {expected:?}"
        );
    }
}

#[test]
fn glb_normals_follow_rigid_rotation() {
    // Rotate 90 degrees around X: (x, y, z) -> (x, -z, y).
    let glb = field_glb(vec![
        [0.0; 3],
        [0.0; 3],
        [0.0, -1.0, 1.0],
        [0.0, -1.0, -1.0],
    ]);
    assert_eq!(
        &attribute(&glb, "POSITION")[..3],
        &[[0.0; 3], [0.0, 0.0, 1.0], [1.0, 0.0, 0.0]]
    );
    for normal in &attribute(&glb, "NORMAL")[..3] {
        assert_normal(*normal, [0.0, 1.0, 0.0]);
    }
}

#[test]
fn glb_normals_follow_nonuniform_displacement() {
    // Raise just the Y vertex; the first face now lies in z = y.
    let glb = field_glb(vec![[0.0; 3], [0.0; 3], [0.0, 0.0, 1.0], [0.0; 3]]);
    let h = std::f32::consts::FRAC_1_SQRT_2;
    for normal in &attribute(&glb, "NORMAL")[..3] {
        assert_normal(*normal, [0.0, h, -h]);
    }
}

#[test]
fn glb_normals_preserve_translation_and_missing_displacement_fallback() {
    for displacements in [vec![[2.0, -3.0, 4.0]; 4], vec![], vec![[0.0; 3]]] {
        let glb = field_glb(displacements);
        for normal in &attribute(&glb, "NORMAL")[..3] {
            assert_normal(*normal, [0.0, 0.0, -1.0]);
        }
    }
}
