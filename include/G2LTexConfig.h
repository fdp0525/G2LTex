/**
  * @author fyp
  */


#ifndef G2LTEXCONFIG_H
#define G2LTEXCONFIG_H

#include <iostream>
#include <opencv2/opencv.hpp>

class G2LTexConfig
{
public:
    static const G2LTexConfig & get()
    {
        static const G2LTexConfig instance;
        return instance;
    }



public:
    float IMAGE_FX;
    float IMAGE_FY;
    float IMAGE_CX;
    float IMAGE_CY;
    float DEPTH_FX;
    float DEPTH_FY;
    float DEPTH_CX;
    float DEPTH_CY;
    int    DEPTH_WIDTH;
    int    DEPTH_HEIGHT;
    int    IMAGE_WIDTH;
    int    IMAGE_HEIGHT;

    int  GLOBAL_ITER_NUM;
    int  LOCAL_ITER_NUM;

    int  MIN_CHART_NUM;
    int  SEARCH_OFFSET;
    int  BOARD_IGNORE;

    bool global_color_correcting;
    bool write_view_selection_model;

    //weights for global texture optimization
    float global_depth_weight;
    float global_RGB_weight;

    //weights for local texture optimization
   float local_RGB_weight;
   float local_edge_wight;
   float local_Reg_wight;

private:
    G2LTexConfig()
        : global_depth_weight(100),
          global_RGB_weight(1),
          local_RGB_weight(1),
          local_edge_wight(1),
          local_Reg_wight(100),
          global_color_correcting(false),
          write_view_selection_model(true),
          GLOBAL_ITER_NUM(100),
          LOCAL_ITER_NUM(5)
    {
        cv::FileStorage  fs2("../Config/config.yml", cv::FileStorage::READ);

        if(!fs2.isOpened())
        {
            std::cout << "Failed to open settings file at: "  << std::endl;
        }

    //depth
        fs2["depth_fx"]>>DEPTH_FX;
        fs2["depth_fy"]>>DEPTH_FY;
        fs2["depth_cx"]>>DEPTH_CX;
        fs2["depth_cy"]>>DEPTH_CY;
        fs2["depth_width"]>>DEPTH_WIDTH;
        fs2["depth_height"]>>DEPTH_HEIGHT;
        std::cout<<"depth fx:"<<DEPTH_FX<<" fy:"<<DEPTH_FY<<" cx:"<<DEPTH_CX<<" cy:"<<DEPTH_CY<<" width:"<<DEPTH_WIDTH<<" height:"<<DEPTH_HEIGHT<<std::endl;

       //rgb
        fs2["RGB_fx"]>>IMAGE_FX;
        fs2["RGB_fy"]>>IMAGE_FY;
        fs2["RGB_cx"]>>IMAGE_CX;
        fs2["RGB_cy"]>>IMAGE_CY;
        fs2["RGB_width"]>>IMAGE_WIDTH;
        fs2["RGB_height"]>>IMAGE_HEIGHT;
        std::cout<<"RGB fx:"<<IMAGE_FX<<" fy:"<<IMAGE_FY<<" cx:"<<IMAGE_CX<<" cy:"<<IMAGE_CY<<" width:"<<IMAGE_WIDTH<<" height:"<<IMAGE_HEIGHT<<std::endl;

        fs2["global_depth_weight"]>>global_depth_weight;
        fs2["global_RGB_weight"]>>global_RGB_weight;

        fs2["local_RGB_weight"]>>local_RGB_weight;
        fs2["local_edge_wight"]>>local_edge_wight;
        fs2["local_Reg_wight"]>>local_Reg_wight;
        std::cout<<"Global depth:"<<global_depth_weight<<" RGB:"<<global_RGB_weight<<std::endl;
        std::cout<<"Local RGB:"<<local_RGB_weight<<" edge:"<<local_edge_wight<<"  Reg:"<<local_Reg_wight<<std::endl;
        fs2["Global_Iterations"]>>GLOBAL_ITER_NUM;
        fs2["Lobal_Iterations"]>>LOCAL_ITER_NUM;
        std::cout<<"Global Iterations:"<<GLOBAL_ITER_NUM<<" Local Iterations:"<<LOCAL_ITER_NUM<<std::endl;

        int wf,gc;
        fs2["write_view_selection_model"]>>wf;
        fs2["global_color_correcting"]>>gc;

        if(1 == wf)
        {
            write_view_selection_model = true;
        }
        else
        {
            write_view_selection_model = false;
        }

        if( 1 == gc)
        {
            global_color_correcting = true;
        }
        else
        {
            global_color_correcting = false;
        }

        MIN_CHART_NUM = 50;

        SEARCH_OFFSET = 10;
        BOARD_IGNORE = 20;

        fs2.release();
    }
};
#endif // G2LTEXCONFIG_H
