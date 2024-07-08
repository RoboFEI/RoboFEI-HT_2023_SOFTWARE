#include <iostream>
class motorsAttributes
{
public:
    int PROTOCOL_VERSION        = 2;

    int ADDR_OPERATING_MODE     = 11;     
    int ADDR_TORQUE_ENABLE      = 64;     
    int ADDR_PROFILE_VELOCITY   = 112;    
    int ADDR_GOAL_POSITION      = 116;    
    int ADDR_PRESENT_POSITION   = 132;    

    int LEN_PROFILE_VELOCITY    = 4;
    int LEN_GOAL_POSITION       = 4;
    int LEN_PRESENT_POSITION    = 4;

    motorsAttributes(int robotNumber);
    ~motorsAttributes();
};

motorsAttributes::motorsAttributes(int robotNumber)
{
    if(robotNumber > 2)
    {
        std::cout << "Protocolo 1" << std::endl;
        PROTOCOL_VERSION        = 1;

        ADDR_TORQUE_ENABLE      = 24;      
        ADDR_PROFILE_VELOCITY   = 32;      
        ADDR_GOAL_POSITION      = 30;       
        ADDR_PRESENT_POSITION   = 36; 

        LEN_PROFILE_VELOCITY    = 2;
        LEN_GOAL_POSITION       = 2;
        LEN_PRESENT_POSITION    = 2;        
    }

}

motorsAttributes::~motorsAttributes()
{
}
