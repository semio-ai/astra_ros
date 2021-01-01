uint32 PIXEL_FORMAT_UNKNOWN = 0
uint32 PIXEL_FORMAT_DEPTH_MM = 100
uint32 PIXEL_FORMAT_RGB888 = 200
uint32 PIXEL_FORMAT_YUV422 = 201
uint32 PIXEL_FORMAT_YUVV = 202
uint32 PIXEL_FORMAT_RGBA = 203
uint32 PIXEL_FORMAT_NV21 = 204
uint32 PIXEL_FORMAT_GRAY8 = 300
uint32 PIXEL_FORMAT_GRAY16 = 301
uint32 PIXEL_FORMAT_POINT = 400

# The width of the mode. 0 means unspecified.
uint32 width

# The height of the mode. 0 means unspecified.
uint32 height

# The frames per second of the mode. 0 means unspecified.
uint8 fps

# The pixel format of the mode. PIXEL_FORMAT_UNKNOWN (0) means unspecified.
uint32 pixel_format