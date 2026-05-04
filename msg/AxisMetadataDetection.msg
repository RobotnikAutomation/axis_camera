std_msgs/Header header

# Track ID from Axis metadata stream
string track_id

# Class label (e.g. "human", "vehicle", "unknown")
string class_label

# Detection confidence [0.0 - 1.0], may be 0.0 when unavailable
float32 score

# Bounding box in normalized coordinates [0.0 - 1.0]
float32 left
float32 top
float32 right
float32 bottom
