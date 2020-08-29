//
// Created by jixingwu on 2020/7/20.
//

#ifndef SRC_CONFIG_H
#define SRC_CONFIG_H

#include "camera.h"

class Config
{
private:
    static shared_ptr<Config> config_;
    cv::FileStorage file_;

    Config() {}

public:
    ~Config();

    // set a new config file
    static void setParameterFile(const std::string& filename);

    // access the parameter values
    template< typename T>
    static T get(const std::string& key)
    {
        return T(Config::config_->file_[key]);
    }
};

#endif //SRC_CONFIG_H
