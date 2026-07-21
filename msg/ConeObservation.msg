uint8 SOURCE_LIDAR=1
uint8 SOURCE_CAMERA=2
uint8 SOURCE_FUSED=3

uint8 SEMANTIC_UNKNOWN=0
uint8 SEMANTIC_BLUE=1
uint8 SEMANTIC_YELLOW=2
uint8 SEMANTIC_SMALL_ORANGE=3
uint8 SEMANTIC_BIG_ORANGE=4

uint8 CANDIDATE_WEAK=1
uint8 CANDIDATE_STRONG=2

std_msgs/Header header
uint32 id
geometry_msgs/Point position
float64[9] position_covariance
uint8 source
uint8 semantic_class
uint8 candidate_level
float32 existence_probability
float32 lidar_confidence
float32 camera_color_confidence
float32 height
float32 width
float32 depth
uint32 point_count
uint8 band_index
bool confirmed
