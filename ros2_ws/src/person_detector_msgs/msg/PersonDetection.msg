# 单个人体检测结果
std_msgs/Header header

# 跟踪ID（由跟踪算法分配，-1表示未跟踪）
int32 track_id

# 检测框（像素坐标）
int32 x1
int32 y1
int32 x2
int32 y2

# 中心点（像素坐标）
int32 center_x
int32 center_y

# 检测置信度 [0.0, 1.0]
float32 confidence

# 深度距离（米）
float32 distance

# 检测框尺寸
int32 width
int32 height

# 是否有有效深度值
bool has_depth
