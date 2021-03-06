/* Automatically generated nanopb header */
/* Generated by nanopb-0.3.9.2 at Sat Dec 29 16:04:41 2018. */

#ifndef PB_DRONE_PB_H_INCLUDED
#define PB_DRONE_PB_H_INCLUDED
#include <pb.h>

/* @@protoc_insertion_point(includes) */
#if PB_PROTO_HEADER_VERSION != 30
#error Regenerate this file with the current version of nanopb generator.
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Struct definitions */
typedef struct _DroneState {
    float roll;
    float pitch;
    float yaw;
    int32_t acc_x;
    int32_t acc_y;
    int32_t acc_z;
    int32_t gyro_x;
    int32_t gyro_y;
    int32_t gyro_z;
    int32_t mag_x;
    int32_t mag_y;
    int32_t mag_z;
/* @@protoc_insertion_point(struct:DroneState) */
} DroneState;

typedef struct _Motors {
    uint32_t motor_DL;
    uint32_t motor_DR;
    uint32_t motor_R;
    uint32_t motor_L;
/* @@protoc_insertion_point(struct:Motors) */
} Motors;

/* Default values for struct fields */

/* Initializer values for message structs */
#define Motors_init_default                      {0, 0, 0, 0}
#define DroneState_init_default                  {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
#define Motors_init_zero                         {0, 0, 0, 0}
#define DroneState_init_zero                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}

/* Field tags (for use in manual encoding/decoding) */
#define DroneState_roll_tag                      1
#define DroneState_pitch_tag                     2
#define DroneState_yaw_tag                       3
#define DroneState_acc_x_tag                     4
#define DroneState_acc_y_tag                     5
#define DroneState_acc_z_tag                     6
#define DroneState_gyro_x_tag                    7
#define DroneState_gyro_y_tag                    8
#define DroneState_gyro_z_tag                    9
#define DroneState_mag_x_tag                     10
#define DroneState_mag_y_tag                     11
#define DroneState_mag_z_tag                     12
#define Motors_motor_DL_tag                      1
#define Motors_motor_DR_tag                      2
#define Motors_motor_R_tag                       3
#define Motors_motor_L_tag                       4

/* Struct field encoding specification for nanopb */
extern const pb_field_t Motors_fields[5];
extern const pb_field_t DroneState_fields[13];

/* Maximum encoded size of messages (where known) */
#define Motors_size                              24
#define DroneState_size                          114

/* Message IDs (where set with "msgid" option) */
#ifdef PB_MSGID

#define DRONE_MESSAGES \


#endif

#ifdef __cplusplus
} /* extern "C" */
#endif
/* @@protoc_insertion_point(eof) */

#endif
