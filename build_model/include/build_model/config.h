#ifndef CONFIG_H
#define CONFIG_H

#include "common_include.h"

using namespace std;

namespace buildmodel
{
class Config
{
private:
    static std::shared_ptr<Config> _config;
    cv::FileStorage _file;
    
    Config () {} // private constructor makes a singleton
public:
    ~Config();  // close the file when deconstructing 
    
    // set a new config file 
    static void setParameterFile( const std::string& filename ); 

    // mzm add,set a default file
    static void setParameterFile();
    
    // access the parameter values
    template< typename T >
    static T get( const std::string& key )
    {
        return T( Config::_config->_file[key] );
    }
};
}

#endif // CONFIG_H
