
#include <vector>
#include <string>
#include <iostream>

typedef struct _videodev_t{
    std::string dev_name;
    std::string media_name;
    std::string channel;
    std::string id;
} videodev_t;

std::vector<videodev_t> query_device_status(const std::string& device_type);
int media_init ( videodev_t * video, int width, int height);
