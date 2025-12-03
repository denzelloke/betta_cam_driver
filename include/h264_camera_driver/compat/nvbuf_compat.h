// Copyright by BeeX [2024]
// Compatibility layer for migrating from nvbuf_utils to NvUtils API
// For L4T 35.4.1 (JetPack 5.x)
#ifndef NVBUF_COMPAT_H
#define NVBUF_COMPAT_H

#include <nvbufsurface.h>
#include <nvbufsurftransform.h>
#include <cstring>
#include <mutex>
#include <unordered_map>

// ============================================================================
// Legacy enum mappings
// ============================================================================

// Layout formats
#define NvBufferLayout_Pitch       NVBUF_LAYOUT_PITCH
#define NvBufferLayout_BlockLinear NVBUF_LAYOUT_BLOCK_LINEAR

// Memory flags
#define NvBufferMem_Read       NVBUF_MAP_READ
#define NvBufferMem_Write      NVBUF_MAP_WRITE
#define NvBufferMem_Read_Write NVBUF_MAP_READ_WRITE

// Payload/Memory types
#define NvBufferPayload_SurfArray NVBUF_MEM_SURFACE_ARRAY
#define NvBufferPayload_MemHandle NVBUF_MEM_HANDLE

// Buffer tags
#define NvBufferTag_NONE          NvBufSurfaceTag_NONE
#define NvBufferTag_CAMERA        NvBufSurfaceTag_CAMERA
#define NvBufferTag_JPEG          NvBufSurfaceTag_JPEG
#define NvBufferTag_VIDEO_ENC     NvBufSurfaceTag_VIDEO_ENC
#define NvBufferTag_VIDEO_DEC     NvBufSurfaceTag_VIDEO_DEC
#define NvBufferTag_VIDEO_CONVERT NvBufSurfaceTag_VIDEO_CONVERT

// Transform flags
#define NVBUFFER_TRANSFORM_CROP_SRC NVBUFSURF_TRANSFORM_CROP_SRC
#define NVBUFFER_TRANSFORM_CROP_DST NVBUFSURF_TRANSFORM_CROP_DST
#define NVBUFFER_TRANSFORM_FILTER   NVBUFSURF_TRANSFORM_FILTER
#define NVBUFFER_TRANSFORM_FLIP     NVBUFSURF_TRANSFORM_FLIP

// Transform flip methods
#define NvBufferTransform_None         NvBufSurfTransform_None
#define NvBufferTransform_Rotate90     NvBufSurfTransform_Rotate90
#define NvBufferTransform_Rotate180    NvBufSurfTransform_Rotate180
#define NvBufferTransform_Rotate270    NvBufSurfTransform_Rotate270
#define NvBufferTransform_FlipX        NvBufSurfTransform_FlipX
#define NvBufferTransform_FlipY        NvBufSurfTransform_FlipY
#define NvBufferTransform_Transpose    NvBufSurfTransform_Transpose
#define NvBufferTransform_InvTranspose NvBufSurfTransform_InvTranspose

// Transform filter types
#define NvBufferTransform_Filter_Nearest  NvBufSurfTransformInter_Nearest
#define NvBufferTransform_Filter_Bilinear NvBufSurfTransformInter_Bilinear
#define NvBufferTransform_Filter_5_Tap    NvBufSurfTransformInter_Algo1
#define NvBufferTransform_Filter_10_Tap   NvBufSurfTransformInter_Algo2
#define NvBufferTransform_Filter_Smart    NvBufSurfTransformInter_Algo3
#define NvBufferTransform_Filter_Nicest   NvBufSurfTransformInter_Algo4

// Color formats
#define NvBufferColorFormat_YUV420           NVBUF_COLOR_FORMAT_YUV420
#define NvBufferColorFormat_YVU420           NVBUF_COLOR_FORMAT_YVU420
#define NvBufferColorFormat_YUV422           NVBUF_COLOR_FORMAT_YUV422
#define NvBufferColorFormat_YUV420_ER        NVBUF_COLOR_FORMAT_YUV420_ER
#define NvBufferColorFormat_YVU420_ER        NVBUF_COLOR_FORMAT_YVU420_ER
#define NvBufferColorFormat_NV12             NVBUF_COLOR_FORMAT_NV12
#define NvBufferColorFormat_NV12_ER          NVBUF_COLOR_FORMAT_NV12_ER
#define NvBufferColorFormat_NV21             NVBUF_COLOR_FORMAT_NV21
#define NvBufferColorFormat_NV21_ER          NVBUF_COLOR_FORMAT_NV21_ER
#define NvBufferColorFormat_UYVY             NVBUF_COLOR_FORMAT_UYVY
#define NvBufferColorFormat_UYVY_ER          NVBUF_COLOR_FORMAT_UYVY_ER
#define NvBufferColorFormat_VYUY             NVBUF_COLOR_FORMAT_VYUY
#define NvBufferColorFormat_VYUY_ER          NVBUF_COLOR_FORMAT_VYUY_ER
#define NvBufferColorFormat_YUYV             NVBUF_COLOR_FORMAT_YUYV
#define NvBufferColorFormat_YUYV_ER          NVBUF_COLOR_FORMAT_YUYV_ER
#define NvBufferColorFormat_YVYU             NVBUF_COLOR_FORMAT_YVYU
#define NvBufferColorFormat_YVYU_ER          NVBUF_COLOR_FORMAT_YVYU_ER
#define NvBufferColorFormat_ABGR32           NVBUF_COLOR_FORMAT_RGBA
#define NvBufferColorFormat_XRGB32           NVBUF_COLOR_FORMAT_BGRx
#define NvBufferColorFormat_ARGB32           NVBUF_COLOR_FORMAT_BGRA
#define NvBufferColorFormat_NV12_10LE        NVBUF_COLOR_FORMAT_NV12_10LE
#define NvBufferColorFormat_NV12_10LE_709    NVBUF_COLOR_FORMAT_NV12_10LE_709
#define NvBufferColorFormat_NV12_10LE_709_ER NVBUF_COLOR_FORMAT_NV12_10LE_709_ER
#define NvBufferColorFormat_NV12_10LE_2020   NVBUF_COLOR_FORMAT_NV12_10LE_2020
#define NvBufferColorFormat_NV21_10LE        NVBUF_COLOR_FORMAT_NV21_10LE
#define NvBufferColorFormat_NV12_12LE        NVBUF_COLOR_FORMAT_NV12_12LE
#define NvBufferColorFormat_NV12_12LE_2020   NVBUF_COLOR_FORMAT_NV12_12LE_2020
#define NvBufferColorFormat_NV21_12LE        NVBUF_COLOR_FORMAT_NV21_12LE
#define NvBufferColorFormat_YUV420_709       NVBUF_COLOR_FORMAT_YUV420_709
#define NvBufferColorFormat_YUV420_709_ER    NVBUF_COLOR_FORMAT_YUV420_709_ER
#define NvBufferColorFormat_NV12_709         NVBUF_COLOR_FORMAT_NV12_709
#define NvBufferColorFormat_NV12_709_ER      NVBUF_COLOR_FORMAT_NV12_709_ER
#define NvBufferColorFormat_YUV420_2020      NVBUF_COLOR_FORMAT_YUV420_2020
#define NvBufferColorFormat_NV12_2020        NVBUF_COLOR_FORMAT_NV12_2020
#define NvBufferColorFormat_GRAY8            NVBUF_COLOR_FORMAT_GRAY8
#define NvBufferColorFormat_Invalid          NVBUF_COLOR_FORMAT_INVALID

// ============================================================================
// Legacy type definitions (typedef to new types where possible)
// ============================================================================

typedef NvBufSurfaceLayout NvBufferLayout;
typedef NvBufSurfaceColorFormat NvBufferColorFormat;
typedef NvBufSurfaceMemType NvBufferPayloadType;
typedef NvBufSurfaceTag NvBufferTag;
typedef NvBufSurfaceMemMapFlags NvBufferMemFlags;
typedef NvBufSurfTransform_Flip NvBufferTransform_Flip;
typedef NvBufSurfTransform_Inter NvBufferTransform_Filter;

// Legacy NvBufferCreateParams structure
struct NvBufferCreateParams {
    int32_t width;
    int32_t height;
    int32_t memsize;  // For NvBufferPayload_MemHandle
    NvBufferLayout layout;
    NvBufferColorFormat colorFormat;
    NvBufferPayloadType payloadType;
    NvBufferTag nvbuf_tag;
};

// Legacy NvBufferParams structure
struct NvBufferParams {
    uint32_t dmabuf_fd;
    uint32_t num_planes;
    uint32_t width[NVBUF_MAX_PLANES];
    uint32_t height[NVBUF_MAX_PLANES];
    uint32_t pitch[NVBUF_MAX_PLANES];
    uint32_t offset[NVBUF_MAX_PLANES];
    uint32_t psize[NVBUF_MAX_PLANES];
    NvBufferColorFormat pixel_format;
    NvBufferPayloadType payloadType;
};

// Legacy NvBufferRect structure
struct NvBufferRect {
    uint32_t top;
    uint32_t left;
    uint32_t width;
    uint32_t height;
};

// Legacy NvBufferTransformParams structure
struct NvBufferTransformParams {
    uint32_t transform_flag;
    NvBufferTransform_Flip transform_flip;
    NvBufferTransform_Filter transform_filter;
    NvBufferRect src_rect;
    NvBufferRect dst_rect;
};

// ============================================================================
// Internal buffer registry (maps dmabuf_fd to NvBufSurface*)
// ============================================================================

namespace NvBufCompat {

class BufferRegistry {
public:
    static BufferRegistry &instance() {
        static BufferRegistry inst;
        return inst;
    }

    void registerBuffer(int32_t fd, NvBufSurface *surf) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffers_[fd] = surf;
    }

    void unregisterBuffer(int32_t fd) {
        std::lock_guard<std::mutex> lock(mutex_);
        buffers_.erase(fd);
    }

    NvBufSurface *getSurface(int32_t fd) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto it = buffers_.find(fd);
        return (it != buffers_.end()) ? it->second : nullptr;
    }

private:
    BufferRegistry() = default;
    std::mutex mutex_;
    std::unordered_map<int32_t, NvBufSurface *> buffers_;
};

}  // namespace NvBufCompat

// ============================================================================
// Legacy API function implementations
// ============================================================================

/**
 * @brief Creates a hardware buffer (legacy API compatibility)
 * @param dmabuf_fd Output: Returns the DMABUF FD of the hardware buffer
 * @param input_params Input parameters for hardware buffer creation
 * @return 0 for success, -1 for failure
 */
inline int NvBufferCreateEx(int* dmabuf_fd, NvBufferCreateParams* input_params) {
    if (!dmabuf_fd || !input_params) {
        return -1;
    }

    NvBufSurfaceAllocateParams allocParams;
    memset(&allocParams, 0, sizeof(allocParams));

    allocParams.params.width = input_params->width;
    allocParams.params.height = input_params->height;
    allocParams.params.layout = static_cast<NvBufSurfaceLayout>(input_params->layout);
    allocParams.params.colorFormat = static_cast<NvBufSurfaceColorFormat>(input_params->colorFormat);
    allocParams.params.memType = static_cast<NvBufSurfaceMemType>(input_params->payloadType);
    allocParams.memtag = static_cast<NvBufSurfaceTag>(input_params->nvbuf_tag);

    if (input_params->memsize > 0) {
        allocParams.params.size = input_params->memsize;
    }

    NvBufSurface* surf = nullptr;
    if (NvBufSurfaceAllocate(&surf, 1, &allocParams) != 0) {
        return -1;
    }

    // *** CRITICAL FIX: Set numFilled to indicate buffer is valid ***
    surf->numFilled = 1;

    *dmabuf_fd = surf->surfaceList[0].bufferDesc;
    NvBufCompat::BufferRegistry::instance().registerBuffer(*dmabuf_fd, surf);

    return 0;
}

/**
 * @brief Destroys a hardware buffer (legacy API compatibility)
 * @param dmabuf_fd The dmabuf_fd of the buffer to destroy
 * @return 0 for success, -1 for failure
 */
inline int NvBufferDestroy(int dmabuf_fd) {
    NvBufSurface *surf = NvBufCompat::BufferRegistry::instance().getSurface(dmabuf_fd);
    if (!surf) {
        // Buffer not in registry - might be externally managed
        return 0;
    }

    NvBufCompat::BufferRegistry::instance().unregisterBuffer(dmabuf_fd);
    return NvBufSurfaceDestroy(surf);
}

/**
 * @brief Gets buffer parameters (legacy API compatibility)
 * @param dmabuf_fd The dmabuf_fd of the buffer
 * @param params Output: Buffer parameters
 * @return 0 for success, -1 for failure
 */
inline int NvBufferGetParams(int dmabuf_fd, NvBufferParams *params) {
    if (!params) {
        return -1;
    }

    NvBufSurface *surf = NvBufCompat::BufferRegistry::instance().getSurface(dmabuf_fd);
    if (!surf || surf->numFilled < 1) {
        return -1;
    }

    NvBufSurfaceParams *surfParams = &surf->surfaceList[0];

    params->dmabuf_fd    = surfParams->bufferDesc;
    params->num_planes   = surfParams->planeParams.num_planes;
    params->pixel_format = surfParams->colorFormat;
    params->payloadType  = surf->memType;

    for (uint32_t i = 0; i < surfParams->planeParams.num_planes && i < NVBUF_MAX_PLANES; i++) {
        params->width[i]  = surfParams->planeParams.width[i];
        params->height[i] = surfParams->planeParams.height[i];
        params->pitch[i]  = surfParams->planeParams.pitch[i];
        params->offset[i] = surfParams->planeParams.offset[i];
        params->psize[i]  = surfParams->planeParams.psize[i];
    }

    return 0;
}

/**
 * @brief Maps buffer memory (legacy API compatibility)
 * @param dmabuf_fd The dmabuf_fd of the buffer
 * @param plane Plane index to map
 * @param memflag Memory access flags
 * @param pVirtAddr Output: Virtual address pointer
 * @return 0 for success, -1 for failure
 */
inline int NvBufferMemMap(int dmabuf_fd, unsigned int plane, NvBufferMemFlags memflag, void **pVirtAddr) {
    if (!pVirtAddr) {
        return -1;
    }

    NvBufSurface *surf = NvBufCompat::BufferRegistry::instance().getSurface(dmabuf_fd);
    if (!surf) {
        return -1;
    }

    NvBufSurfaceMemMapFlags mapFlags = static_cast<NvBufSurfaceMemMapFlags>(memflag);
    if (NvBufSurfaceMap(surf, 0, plane, mapFlags) != 0) {
        return -1;
    }

    *pVirtAddr = surf->surfaceList[0].mappedAddr.addr[plane];
    return 0;
}

/**
 * @brief Unmaps buffer memory (legacy API compatibility)
 * @param dmabuf_fd The dmabuf_fd of the buffer
 * @param plane Plane index to unmap
 * @param pVirtAddr Virtual address pointer (unused in new API)
 * @return 0 for success, -1 for failure
 */
inline int NvBufferMemUnMap(int dmabuf_fd, unsigned int plane, void **pVirtAddr) {
    (void)pVirtAddr;  // Not used in new API

    NvBufSurface *surf = NvBufCompat::BufferRegistry::instance().getSurface(dmabuf_fd);
    if (!surf) {
        return -1;
    }

    return NvBufSurfaceUnMap(surf, 0, plane);
}

/**
 * @brief Syncs buffer for CPU access (legacy API compatibility)
 * @param dmabuf_fd The dmabuf_fd of the buffer
 * @param plane Plane index
 * @param pVirtAddr Virtual address pointer (unused in new API)
 * @return 0 for success, -1 for failure
 */
inline int NvBufferMemSyncForCpu(int dmabuf_fd, unsigned int plane, void **pVirtAddr) {
    (void)pVirtAddr;

    NvBufSurface *surf = NvBufCompat::BufferRegistry::instance().getSurface(dmabuf_fd);
    if (!surf) {
        return -1;
    }

    return NvBufSurfaceSyncForCpu(surf, 0, plane);
}

/**
 * @brief Syncs buffer for device access (legacy API compatibility)
 * @param dmabuf_fd The dmabuf_fd of the buffer
 * @param plane Plane index
 * @param pVirtAddr Virtual address pointer (unused in new API)
 * @return 0 for success, -1 for failure
 */
inline int NvBufferMemSyncForDevice(int dmabuf_fd, unsigned int plane, void **pVirtAddr) {
    (void)pVirtAddr;

    NvBufSurface *surf = NvBufCompat::BufferRegistry::instance().getSurface(dmabuf_fd);
    if (!surf) {
        return -1;
    }

    return NvBufSurfaceSyncForDevice(surf, 0, plane);
}

/**
 * @brief Transforms one buffer to another (legacy API compatibility)
 * @param src_dmabuf_fd Source buffer dmabuf_fd
 * @param dst_dmabuf_fd Destination buffer dmabuf_fd
 * @param transform_params Transform parameters
 * @return 0 for success, -1 for failure
 */
inline int NvBufferTransform(int src_dmabuf_fd, int dst_dmabuf_fd, NvBufferTransformParams *transform_params) {
    if (!transform_params) {
        return -1;
    }

    NvBufSurface *srcSurf = NvBufCompat::BufferRegistry::instance().getSurface(src_dmabuf_fd);
    NvBufSurface *dstSurf = NvBufCompat::BufferRegistry::instance().getSurface(dst_dmabuf_fd);

    if (!srcSurf || !dstSurf) {
        return -1;
    }

    // Convert legacy rect to new rect
    NvBufSurfTransformRect srcRect = {
            transform_params->src_rect.top,
            transform_params->src_rect.left,
            transform_params->src_rect.width,
            transform_params->src_rect.height};

    NvBufSurfTransformRect dstRect = {
            transform_params->dst_rect.top,
            transform_params->dst_rect.left,
            transform_params->dst_rect.width,
            transform_params->dst_rect.height};

    NvBufSurfTransformParams newParams;
    memset(&newParams, 0, sizeof(newParams));
    newParams.transform_flag   = transform_params->transform_flag;
    newParams.transform_flip   = static_cast<NvBufSurfTransform_Flip>(transform_params->transform_flip);
    newParams.transform_filter = static_cast<NvBufSurfTransform_Inter>(transform_params->transform_filter);
    newParams.src_rect         = &srcRect;
    newParams.dst_rect         = &dstRect;

    NvBufSurfTransform_Error err = NvBufSurfTransform(srcSurf, dstSurf, &newParams);
    return (err == NvBufSurfTransformError_Success) ? 0 : -1;
}

#endif  // NVBUF_COMPAT_H