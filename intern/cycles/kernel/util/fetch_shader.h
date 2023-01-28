/* SPDX-License-Identifier: Apache-2.0
 * Copyright 2011-2022 Blender Foundation */

/* Fetch shader from object's used shaders */

#pragma once

CCL_NAMESPACE_BEGIN

/* Fetch shader from tri_shader_index. */
ccl_device_inline uint triangle_fetch_shader(KernelGlobals kg, int object, int prim)
{
  const int index = kernel_data_fetch(tri_shader_index, prim);
  if ((index & ~(SHADER_COMPACT_SMOOTH_NORMAL)) - 1 == SHADER_NONE)
    return (index & SHADER_COMPACT_SMOOTH_NORMAL) ? (SHADER_DEFAULT_SHADER | SHADER_SMOOTH_NORMAL) : SHADER_DEFAULT_SHADER;
  const int used_shaders_offset = kernel_data_fetch(objects, object).used_shaders_offset;
  const uint shader = kernel_data_fetch(object_used_shaders, used_shaders_offset + (index & ~(SHADER_COMPACT_SMOOTH_NORMAL)) - 1);
  return (index & SHADER_COMPACT_SMOOTH_NORMAL) ? (shader | SHADER_SMOOTH_NORMAL) : shader;
}

CCL_NAMESPACE_END
