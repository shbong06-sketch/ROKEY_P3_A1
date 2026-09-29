from pxr import Usd, UsdGeom


def triangulate_mesh(mesh_prim: Usd.Prim) -> bool:
    mesh = UsdGeom.Mesh(mesh_prim)
    counts = list(mesh.GetFaceVertexCountsAttr().Get() or [])
    indices = list(mesh.GetFaceVertexIndicesAttr().Get() or [])

    if not counts:
        print(f"[건너뜀] {mesh_prim.GetPath()}: face 데이터 없음")
        return False
    if all(c == 3 for c in counts):
        print(f"[정보] {mesh_prim.GetPath()}: 이미 전부 삼각형입니다.")
        return False

    original_fv_count = len(indices)  # 원본 face-varying 배열의 기준 길이

    new_counts, new_indices = [], []
    fv_map = []            # 새 face-vertex 순서 -> 원본 face-vertex 위치(0..original_fv_count-1)
    new_face_to_old = []   # 새 face 순서 -> 원본 face 인덱스 (uniform primvar용)

    idx_cursor = 0
    for old_face_idx, c in enumerate(counts):
        fv_positions = list(range(idx_cursor, idx_cursor + c))
        face_indices = indices[idx_cursor: idx_cursor + c]
        idx_cursor += c

        if c < 3:
            print(f"[경고] {mesh_prim.GetPath()}: degenerate face(변 {c}개) 건너뜀")
            continue
        if c == 3:
            new_counts.append(3)
            new_indices.extend(face_indices)
            fv_map.extend(fv_positions)
            new_face_to_old.append(old_face_idx)
        else:
            for i in range(1, c - 1):
                new_counts.append(3)
                new_indices.extend([face_indices[0], face_indices[i], face_indices[i + 1]])
                fv_map.extend([fv_positions[0], fv_positions[i], fv_positions[i + 1]])
                new_face_to_old.append(old_face_idx)

    mesh.GetFaceVertexCountsAttr().Set(new_counts)
    mesh.GetFaceVertexIndicesAttr().Set(new_indices)

    # --- normals 재배열 ---
    normals_attr = mesh.GetNormalsAttr()
    if normals_attr.HasValue():
        normals = normals_attr.Get()
        interp = mesh.GetNormalsInterpolation()
        if interp == UsdGeom.Tokens.faceVarying and len(normals) == original_fv_count:
            normals_attr.Set([normals[i] for i in fv_map])
        elif interp == UsdGeom.Tokens.uniform and len(normals) == len(counts):
            normals_attr.Set([normals[i] for i in new_face_to_old])
        elif interp == UsdGeom.Tokens.vertex:
            pass  # 정점 인덱스 기반이라 값 배열 자체는 안 바뀜
        else:
            print(f"[경고] {mesh_prim.GetPath()}: normals 배열 크기가 예상과 달라 "
                  f"재배열을 건너뛰었습니다 — 수동 확인 필요. "
                  f"(interp={interp}, len(normals)={len(normals)}, original_fv_count={original_fv_count})")

    # --- UV(primvars:st) 재배열 ---
    primvars_api = UsdGeom.PrimvarsAPI(mesh_prim)
    st = primvars_api.GetPrimvar("st")
    if st and st.HasValue():
        interp = st.GetInterpolation()
        values = st.Get()
        if st.IsIndexed():
            old_pv_indices = list(st.GetIndices())
            if interp == UsdGeom.Tokens.faceVarying and len(old_pv_indices) == original_fv_count:
                st.SetIndices([old_pv_indices[i] for i in fv_map])
            else:
                print(f"[경고] {mesh_prim.GetPath()}: 인덱스형 UV 재배열 조건이 안 맞아 건너뜀 — 수동 확인 필요. "
                      f"(interp={interp}, len(indices)={len(old_pv_indices)}, original_fv_count={original_fv_count})")
        else:
            if interp == UsdGeom.Tokens.faceVarying and len(values) == original_fv_count:
                st.Set([values[i] for i in fv_map])
            elif interp == UsdGeom.Tokens.uniform and len(values) == len(counts):
                st.Set([values[i] for i in new_face_to_old])
            elif interp == UsdGeom.Tokens.vertex:
                pass
            else:
                print(f"[경고] {mesh_prim.GetPath()}: UV 배열 크기가 예상과 달라 재배열을 건너뛰었습니다.")

    print(f"[삼각형화] {mesh_prim.GetPath()}: face {len(counts)} -> {len(new_counts)}")
    return True
