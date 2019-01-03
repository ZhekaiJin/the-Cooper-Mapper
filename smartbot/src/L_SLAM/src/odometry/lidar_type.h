#ifndef LIDAR_TYPE_DATASHEET_H
#define LIDAR_TYPE_DATASHEET_H

namespace lidar_slam
{
    const int id_reorder_hdl32e[32] = {0,2,4,6,8,10,12,14,16,18,20,22,24,26,28,30,1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31};

    const int id_reorder_pandar[40] = {0,1,2,3,4,5,6,7,8,9,10,
                                         11,12,13,14,15,16,17,18,19,20,
                                         21,22,23,24,25,26,27,28,29,30,
                                         31,32,33,34,35,36,37,38,39};
    const float angle_pandar[40] = {
            6.96,
            5.976,
            4.988,
            3.996,
            2.999,
            2.001,
            1.667,
            1.333,
            1.001,
            0.667,
            0.333,
            0,
            -0.334,
            -0.667,
            -1.001,
            -1.334,
            -1.667,
            -2.001,
            -2.331,
            -2.667,
            -3,
            -3.327,
            -3.663,
            -3.996,
            -4.321,
            -4.657,
            -4.986,
            -5.311,
            -5.647,
            -5.974,
            -6.957,
            -7.934,
            -8.908,
            -9.871,
            -10.826,
            -11.772,
            -12.705,
            -13.63,
            -14.543,
            -15.444
    };


    int scanID_hdl32e(float angle)
    {
        return (int)((angle + 30.66667)/1.33333 + 0.5);
    }

    int scanID_pandar40(float angle) {
        int scanID  = 0;
        for(unsigned i = 0 ; i < 40 ; i++ )
        {
            if(fabs(angle - angle_pandar[i])< 0.01)
            {
                scanID = 39 - i;
            }

//            if(scanID == 10)
//            {
//                std::cout<<angle_pandar[i]<<" "<<abs(angle - angle_pandar[i])<<std::endl;
//            }
        }
        return scanID;
    }

    int scanID_pandar(float angle)
    {
        int scanID = 0;

        if(angle < -15.0 )
        {
            scanID = 0;
        }
        else if(angle > -15.0 && angle < -5.8)
        {
            scanID = (int)(angle + 16.0 + 0.5);
        }
        else if(angle > -5.8 && angle < 2.8)
        {
            scanID = (int)((angle + 5.667) / 0.33 + 0.5) + 10;
        }
        else if(angle > 1.8 && angle < 7.5)
        {
            scanID = (int)(angle - 2.0 + 0.5) + 34;
        }
        else
        {
            std::cout<<"ERROR"<<std::endl;
        }

        return scanID;
    }

}

#endif // LIDAR_TYPE_DATASHEET_H
