/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2022 NVIDIA Corporation
 * Copyright 2022 Blender Foundation */

#include "hydra/attribute.h"
#include "scene/attribute.h"
#include "scene/geometry.h"
#include "scene/scene.h"

#include <pxr/base/gf/vec2f.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/gf/vec4f.h>
#include <pxr/base/vt/array.h>
#include <pxr/imaging/hd/tokens.h>

#include "mikktspace.hh"

HDCYCLES_NAMESPACE_OPEN_SCOPE

template<bool is_subd> struct MikkMeshWrapper {
  MikkMeshWrapper(const Mesh *mesh,
                  const ustring &name,
                  float3 *tangent,
                  float *tangent_sign)
      : mesh(mesh), texface(NULL), orco(NULL), tangent(tangent), tangent_sign(tangent_sign)
  {
    const AttributeSet &attributes = is_subd ? mesh->subd_attributes : mesh->attributes;

    Attribute *attr_vN = attributes.find(ATTR_STD_VERTEX_NORMAL);
    vertex_normal = attr_vN->data_float3();

    Attribute *attr_uv = attributes.find(name);
    if (attr_uv != NULL) {
      texface = attr_uv->data_float2();
    }
  }

  int GetNumFaces()
  {
    if constexpr (is_subd) {
      return mesh->get_num_subd_faces();
    }
    else {
      return mesh->num_triangles();
    }
  }

  int GetNumVerticesOfFace(const int face_num)
  {
    if constexpr (is_subd) {
      return mesh->get_subd_num_corners()[face_num];
    }
    else {
      return 3;
    }
  }

  int CornerIndex(const int face_num, const int vert_num)
  {
    if constexpr (is_subd) {
      const Mesh::SubdFace &face = mesh->get_subd_face(face_num);
      return face.start_corner + vert_num;
    }
    else {
      return face_num * 3 + vert_num;
    }
  }

  int VertexIndex(const int face_num, const int vert_num)
  {
    int corner = CornerIndex(face_num, vert_num);
    if constexpr (is_subd) {
      return mesh->get_subd_face_corners()[corner];
    }
    else {
      return mesh->get_triangles()[corner];
    }
  }

  mikk::float3 GetPosition(const int face_num, const int vert_num)
  {
    const float3 vP = mesh->get_verts()[VertexIndex(face_num, vert_num)];
    return mikk::float3(vP.x, vP.y, vP.z);
  }

  mikk::float3 GetTexCoord(const int face_num, const int vert_num)
  {
    /* TODO: Check whether introducing a template boolean in order to
     * turn this into a constexpr is worth it. */
    if (texface != NULL) {
      const int corner_index = CornerIndex(face_num, vert_num);
      float2 tfuv = texface[corner_index];
      return mikk::float3(tfuv.x, tfuv.y, 1.0f);
    }
    else if (orco != NULL) {
      const int vertex_index = VertexIndex(face_num, vert_num);
      const float2 uv = map_to_sphere((orco[vertex_index] + orco_loc) * inv_orco_size);
      return mikk::float3(uv.x, uv.y, 1.0f);
    }
    else {
      return mikk::float3(0.0f, 0.0f, 1.0f);
    }
  }

  mikk::float3 GetNormal(const int face_num, const int vert_num)
  {
    float3 vN;
    if (is_subd) {
      const Mesh::SubdFace &face = mesh->get_subd_face(face_num);
      if (face.smooth) {
        const int vertex_index = VertexIndex(face_num, vert_num);
        vN = vertex_normal[vertex_index];
      }
      else {
        vN = face.normal(mesh);
      }
    }
    else {
      if (mesh->get_smooth()[face_num]) {
        const int vertex_index = VertexIndex(face_num, vert_num);
        vN = vertex_normal[vertex_index];
      }
      else {
        const Mesh::Triangle tri = mesh->get_triangle(face_num);
        vN = tri.compute_normal(&mesh->get_verts()[0]);
      }
    }
    return mikk::float3(vN.x, vN.y, vN.z);
  }

  void SetTangentSpace(const int face_num, const int vert_num, mikk::float3 T, bool orientation)
  {
    const int corner_index = CornerIndex(face_num, vert_num);
    tangent[corner_index] = make_float3(T.x, T.y, T.z);
    if (tangent_sign != NULL) {
      tangent_sign[corner_index] = orientation ? 1.0f : -1.0f;
    }
  }

  const Mesh *mesh;
  int num_faces;

  float3 *vertex_normal;
  float2 *texface;
  float3 *orco;
  float3 orco_loc, inv_orco_size;

  float3 *tangent;
  float *tangent_sign;
};

void ApplyPrimvars(AttributeSet &attributes,
                   const ustring &name,
                   VtValue value,
                   AttributeElement elem,
                   AttributeStandard std)
{
  const void *data = HdGetValueData(value);
  size_t size = value.GetArraySize();

  const HdType valueType = HdGetValueTupleType(value).type;

  TypeDesc attrType = CCL_NS::TypeUnknown;
  switch (valueType) {
    case HdTypeFloat:
      attrType = CCL_NS::TypeFloat;
      size *= sizeof(float);
      break;
    case HdTypeFloatVec2:
      attrType = CCL_NS::TypeFloat2;
      size *= sizeof(float2);
      static_assert(sizeof(GfVec2f) == sizeof(float2));
      break;
    case HdTypeFloatVec3: {
      attrType = CCL_NS::TypeVector;
      size *= sizeof(float3);
      // The Cycles "float3" data type is padded to "float4", so need to convert the array
      const auto &valueData = value.Get<VtVec3fArray>();
      VtArray<float3> valueConverted;
      valueConverted.reserve(valueData.size());
      for (const GfVec3f &vec : valueData) {
        valueConverted.push_back(make_float3(vec[0], vec[1], vec[2]));
      }
      data = valueConverted.data();
      value = std::move(valueConverted);
      break;
    }
    case HdTypeFloatVec4:
      attrType = CCL_NS::TypeFloat4;
      size *= sizeof(float4);
      static_assert(sizeof(GfVec4f) == sizeof(float4));
      break;
    default:
      TF_WARN("Unsupported attribute type %d", static_cast<int>(valueType));
      return;
  }

  Attribute *const attr = attributes.add(name, attrType, elem);
  attr->std = std;

  assert(size == attr->buffer.size());
  std::memcpy(attr->data(), data, size);
}

void ApplyTangents(Mesh *mesh, const ustring &name, const bool needSign)
{
  /* Create tangent attributes. */
  const bool is_subd = mesh->get_num_subd_faces();
  AttributeSet &attributes = is_subd ? mesh->subd_attributes : mesh->attributes;
  Attribute *attr = attributes.add(ATTR_STD_UV_TANGENT, ustring(string(name.c_str()) + ".tangent"));
  float3 *tangent = attr->data_float3();
  float *tangent_sign = NULL;
  if (needSign) {
    Attribute *attr_sign = attributes.add(ATTR_STD_UV_TANGENT_SIGN, ustring(string(name.c_str()) + ".tangent_sign"));
    tangent_sign = attr_sign->data_float();
  }

  /* Setup userdata. */
  if (is_subd) {
    MikkMeshWrapper<true> userdata(mesh, name, tangent, tangent_sign);
    /* Compute tangents. */
    mikk::Mikktspace(userdata).genTangSpace();
  }
  else {
    MikkMeshWrapper<false> userdata(mesh, name, tangent, tangent_sign);
    /* Compute tangents. */
    mikk::Mikktspace(userdata).genTangSpace();
  }
}

HDCYCLES_NAMESPACE_CLOSE_SCOPE
