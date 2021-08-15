/* Copyright (c) 2017, Yu Zhang, Intelligent Vehicle Research Center(IVRC),
 * Beijing Institute of Technology.
 * All rights reserved.
 *        Files: csv_file.hpp
 *   Created on: 05, 08, 2017
 *       Author: Yu Zhang
 *        Email: yu.zhang.bit@gmail.com
 */
#ifndef OPT_UTILS_CSV_FILE_HPP
#define OPT_UTILS_CSV_FILE_HPP
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

namespace hmpl {
class CSVFile;

inline static CSVFile& endrow(CSVFile& file);
inline static CSVFile& flush(CSVFile& file);

class CSVFile {
    std::ofstream fs_;
    const std::string separator_;

 public:
    CSVFile(const std::string filename, const std::string separator = ",")
        : fs_(), separator_(separator) {
        fs_.exceptions(std::ios::failbit | std::ios::badbit);
        fs_.open(filename);
    }

    ~CSVFile() {
        flush();
        fs_.close();
    }

    void flush() { fs_.flush(); }

    void endrow() { fs_ << std::endl; }

    CSVFile& operator<<(CSVFile& (*val)(CSVFile&)) { return val(*this); }

    CSVFile& operator<<(const char* val) {
        fs_ << '"' << val << '"' << separator_;
        return *this;
    }

    CSVFile& operator<<(const std::string& val) {
        fs_ << '"' << val << '"' << separator_;
        return *this;
    }

    template <typename T>
    CSVFile& operator<<(const T& val) {
        fs_ << val << separator_;
        return *this;
    }
};

inline static CSVFile& endrow(CSVFile& file) {
    file.endrow();
    return file;
}

inline static CSVFile& flush(CSVFile& file) {
    file.flush();
    return file;
}

/*
 * A class to read from a csv file.
 */
class CSVReader {
    std::string file_name_;
    std::string delimeter_;
 
 public:
    CSVReader(std::string file_name, std::string delm = ","):
            file_name_(file_name), delimeter_(delm) {
        this->getData();
    }
    
    std::vector< std::vector<std::string> > data_list;
    // Function to fetch data from a csv file
    std::vector< std::vector<std::string> > getData() {
        std::ifstream file(file_name_);
        this->data_list.clear();
        std::string line = "";
        // Iterate through each lline and split the content using delimeter
        while(std::getline(file, line)) {
            std::vector<std::string> vec;
            boost::algorithm::split(vec, line, boost::is_any_of(delimeter_));
            data_list.push_back(vec);
        }
        // Close the File
        file.close();
        return data_list;
    };
};

}  // namespace hmpl

#endif  // OPT_UTILS_CSV_FILE_HPP
