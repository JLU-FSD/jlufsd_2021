#include <iostream>
#include <vector>
#include "rapidcsv.h"
#include "auto_ros_base_path.h"
int main()
{
    std::string csv_file = auto_ros_base_path + "src/common_tool/test/4d4s_ref.csv";
    rapidcsv::Document doc(csv_file, rapidcsv::LabelParams(0, 0));

    std::vector<double> close = doc.GetColumn<double>("ax");
    for (const auto i : close)
        std::cout << i << ' ';
}