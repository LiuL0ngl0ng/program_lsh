/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC), 
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: csv_test.cpp
 *   Created on: 12, 07, 2017 
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#include "opt_utils/csv_file.hpp"
#include "opt_utils/csv_reader.hpp"
#include "opt_utils/opt_utils.hpp"
#include <gtest/gtest.h>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(CSVTest, writer) {
    std::string file_name = "test.csv";
    hmpl::CSVFile csv("test.csv");
    hmpl::CSVWriter csv_n(",");
    
    csv << "x" << "y" << "z" << hmpl::endrow;
    csv_n.newRow() << "x" << "y" << "z";
    csv << 1.0 << 2.0 << 3.0 << hmpl::endrow;
    csv_n.newRow() << 1.0 << 2.0 << 3.0;
    csv << 3.0 << 2.0 << 1.0 << hmpl::endrow;
    csv_n.newRow() << 3.0 << 2.0 << 1.0;
    csv << 4.0 << 5.0 << 6.0 << hmpl::endrow;
    csv_n.newRow() << 4.0 << 5.0 << 6.0;
    csv_n.writeToFile("new.csv");
    hmpl::CSVReader reader(file_name);
    std::vector<std::vector<std::string> > datalist = reader.getData();
    // print the content of row by row on screen
    for (std::vector<std::string> vec : datalist) {
        for (std::string data: vec) {
            std::cout << data << " ";
        }
        std::cout << std::endl;
    }
   
    
    // read the files
    io::CSVReader<3> test("new.csv");
    test.read_header(io::ignore_extra_column, "x", "y", "z");
    std::string x, y, z;
    while(test.read_row(x, y, z)) {
        std::cout << x << y << z << std::endl;
    }
}
