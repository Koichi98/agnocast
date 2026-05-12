# Sample of a moderately complex message used to exercise type-erased reception.
# Covers: header + nested-message sequence + fixed-size array + string + string array + bool.
std_msgs/Header header
int64 id
string label
bool active
NamedValue[] readings
float64[9] covariance
string[] tags
