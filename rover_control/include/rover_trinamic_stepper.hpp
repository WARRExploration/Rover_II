#include <rover_motor.hpp>

//Opcodes of all TMCL commands that can be used in direct mode
#define TMCL_ROR 1 //rotate right parameter motornumber, velocity
#define TMCL_ROL 2 //rotate left param: mnum, velocity
#define TMCL_MST 3 //stop motor movement
#define TMCL_MVP 4 //move to position, type ABS|REL|COORD, params: mnum, position|offset
#define TMCL_SAP 5 //set axis parameter, params: parameter, mnum, value
#define TMCL_GAP 6 //get axis paramter
#define TMCL_STAP 7 
#define TMCL_RSAP 8 
#define TMCL_SGP 9 //set global parameter
#define TMCL_GGP 10 //get global paramter
#define TMCL_STGP 11 //store global paramter
#define TMCL_RSGP 12 //restore global paramter
#define TMCL_RFS 13 //reference search
#define TMCL_SIO 14 //set digital output to specified value
#define TMCL_GIO 15 //get value of analog/digital input
#define TMCL_SCO 30 //set coordinate param: coordinate_number, motor_number, position
#define TMCL_GCO 31 //get coordinate param: coordinate number, motor number
#define TMCL_CCO 32 //capture coordinate

//Opcodes of TMCL control functions (to be used to run or abort a TMCL program in the module)
#define TMCL_APPL_STOP 128
#define TMCL_APPL_RUN 129
#define TMCL_APPL_RESET 131

//Options for MVP command
#define MVP_ABS 0
#define MVP_REL 1
#define MVP_COORD 2

//Options for RFS command
#define RFS_START 0
#define RFS_STOP 1
#define RFS_STATUS 2

// Booleans
#define FALSE 0
#define TRUE 1

class rover_trinamic_stepper : public rover_motor
{
public:
    rover_trinamic_stepper(std::string joint_name, int can_socket, 
        hardware_interface::JointStateInterface *jnt_state_interface, 
        hardware_interface::PositionJointInterface *jnt_pos_interface);

    int init(struct properties props);
    int receive();
    int send();

    struct properties {
        __uint8_t send_id;
        __uint8_t receive_id;
        __uint8_t steps;
        __uint8_t microsteps;
        __uint8_t bank;

        properties(__uint8_t send_id, __uint8_t receive_id, __uint8_t steps, __uint8_t microsteps, __uint8_t bank){
            this->send_id = send_id;
            this->receive_id = receive_id;
            this->steps = steps
            this->microsteps = microsteps;
            this->bank = bank;
        }
    };

private:
    struct properties props;
};
