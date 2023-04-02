#include <fstream>
#include <string>
#include "nlohmann/json.hpp"

#include "resample.h"

int main()
{
    nlohmann::json json;
    json["root"] = ".";
    json["meshFile"] = "example/Dog.GoZ";
    json["ZBrush"] = nlohmann::json::object();
    json["ZBrush"]["meshHeight"] = 200.0;
    json["ZBrush"]["voxelSize"] = 1.0;
    json["ZBrush"]["offset"] = 5.0;
    json["ZBrush"]["largestOnly"] = true;
    json["ZBrush"]["inverted"] = true;
    json["ZBrush"]["inward"] = false;

    std::ofstream ofs("build/Release/parameters.txt");
    ofs << json.dump(4) << std::endl;
    ofs.close();

    char buf[255];
    char **zData = nullptr;
    resample("build/Release/parameters.txt", 0.0, buf, 0, buf, 0, zData);
    return 0;

    // char buf[255];
    // char **zData = nullptr;
    // resample("/APPLICATIONS/ZBRUSH 2022/ZSTARTUP/ZPlugs64/remeshWithDualContouring_2022/data/parameters.txt", 0.0, buf, 0, buf, 0, zData);
    // return 0;
}
