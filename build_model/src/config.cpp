#include "config.h"

namespace buildmodel
{
    
void Config::setParameterFile( const std::string& filename )
{
    if ( _config == nullptr )
        _config = shared_ptr<Config>(new Config);
    _config->_file = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    if ( _config->_file.isOpened() == false )
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        _config->_file.release();
        return;
    }
}

//mzm add
void Config::setParameterFile()
{
    std::string filename = "/home/mzm/new_sr300_build_model/src/build_model/config/default2.yaml";
    if ( _config == nullptr )
        _config = shared_ptr<Config>(new Config);
    _config->_file = cv::FileStorage( filename.c_str(), cv::FileStorage::READ );
    if ( _config->_file.isOpened() == false )
    {
        std::cerr<<"parameter file "<<filename<<" does not exist."<<std::endl;
        _config->_file.release();
        return;
    }
}

Config::~Config()
{
    if ( _file.isOpened() )
        _file.release();
}

shared_ptr<Config> Config::_config = nullptr;

}
