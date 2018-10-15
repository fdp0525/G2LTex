#ifndef PARAMARGS_H
#define PARAMARGS_H

#include "util/arguments.h"
#include "tex/settings.h"
#include <string>

class paramArgs
{
public:

    std::string in_scene;
    std::string in_mesh;
    std::string out_prefix;

    std::string data_cost_file;
    std::string labeling_file;

    tex::Settings settings;

    bool write_timings;

    std::string  to_string();

public:



};

paramArgs parse_args(int argc, char **argv);
#endif // PARAMARGS_H
