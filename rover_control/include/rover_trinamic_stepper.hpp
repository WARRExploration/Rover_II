#include <rover_motor.hpp>

//Opcodes of all TMCL commands that can be used in direct mode
#define TMCL_ROR 1
#define TMCL_ROL 2
#define TMCL_MST 3
#define TMCL_MVP 4
#define TMCL_SAP 5
#define TMCL_GAP 6
#define TMCL_STAP 7
#define TMCL_RSAP 8
#define TMCL_SGP 9
#define TMCL_GGP 10
#define TMCL_STGP 11
#define TMCL_RSGP 12
#define TMCL_RFS 13
#define TMCL_SIO 14
#define TMCL_GIO 15
#define TMCL_SCO 30
#define TMCL_GCO 31
#define TMCL_CCO 32

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