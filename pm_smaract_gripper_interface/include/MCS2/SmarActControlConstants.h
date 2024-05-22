/**********************************************************************
* Copyright (c) 2022 SmarAct GmbH
*
* File name: SmarActControlConstants.h
*
* THIS  SOFTWARE, DOCUMENTS, FILES AND INFORMATION ARE PROVIDED 'AS IS'
* WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING,
* BUT  NOT  LIMITED  TO,  THE  IMPLIED  WARRANTIES  OF MERCHANTABILITY,
* FITNESS FOR A PURPOSE, OR THE WARRANTY OF NON-INFRINGEMENT.
* THE  ENTIRE  RISK  ARISING OUT OF USE OR PERFORMANCE OF THIS SOFTWARE
* REMAINS WITH YOU.
* IN  NO  EVENT  SHALL  THE  SMARACT  GMBH  BE  LIABLE  FOR ANY DIRECT,
* INDIRECT, SPECIAL, INCIDENTAL, CONSEQUENTIAL OR OTHER DAMAGES ARISING
* OUT OF THE USE OR INABILITY TO USE THIS SOFTWARE.
**********************************************************************/

#ifndef SMARACT_CTL_CONSTANTS_H
#define SMARACT_CTL_CONSTANTS_H

/**********************************************************/
/* GLOBAL DEFINITIONS                                     */
/**********************************************************/

// @constants Global

#define SA_CTL_INFINITE                                     0xffffffff
#define SA_CTL_HOLD_TIME_INFINITE                           -1

#define SA_CTL_FALSE                                        0x00
#define SA_CTL_TRUE                                         0x01
#define SA_CTL_DISABLED                                     0x00
#define SA_CTL_ENABLED                                      0x01
#define SA_CTL_NON_INVERTED                                 0x00
#define SA_CTL_INVERTED                                     0x01
#define SA_CTL_FORWARD_DIRECTION                            0x00
#define SA_CTL_BACKWARD_DIRECTION                           0x01
#define SA_CTL_EITHER_DIRECTION                             0x02

#define SA_CTL_STRING_MAX_LENGTH                            63
#define SA_CTL_REQUEST_ID_MAX_COUNT                         240

// @endconstants Global

/**********************************************************/
/* INTERFACE TYPES                                        */
/**********************************************************/

// @constants InterfaceType

#define SA_CTL_INTERFACE_USB                                0x0001
#define SA_CTL_INTERFACE_ETHERNET                           0x0002

// @endconstants InterfaceType

/**********************************************************/
/* CHANNEL/MODULE TYPES                                   */
/**********************************************************/

// @constants ChannelModuleType

#define SA_CTL_STICK_SLIP_PIEZO_DRIVER                      0x0001
#define SA_CTL_MAGNETIC_DRIVER                              0x0002
#define SA_CTL_PIEZO_SCANNER_DRIVER                         0x0003

// @endconstants ChannelModuleType

/**********************************************************/
/* EVENT TYPES                                            */
/**********************************************************/

// @constants EventType

#define SA_CTL_EVENT_NONE                                   0x0000

// channel events (0x0001 - 0x3fff)
#define SA_CTL_EVENT_MOVEMENT_FINISHED                      0x0001
#define SA_CTL_EVENT_SENSOR_STATE_CHANGED                   0x0002
#define SA_CTL_EVENT_REFERENCE_FOUND                        0x0003
#define SA_CTL_EVENT_FOLLOWING_ERR_LIMIT                    0x0004
#define SA_CTL_EVENT_HOLDING_ABORTED                        0x0005
#define SA_CTL_EVENT_POSITIONER_TYPE_CHANGED                0x0006
#define SA_CTL_EVENT_PHASING_FINISHED                       0x0007
#define SA_CTL_EVENT_AMP_STATE_CHANGED                      0x000a

// module events (0x4000 - 0x7fff)
#define SA_CTL_EVENT_SM_STATE_CHANGED                       0x4000
#define SA_CTL_EVENT_OVER_TEMPERATURE                       0x4001
#define SA_CTL_EVENT_HIGH_VOLTAGE_OVERLOAD                  0x4002  // deprecated
#define SA_CTL_EVENT_POWER_SUPPLY_OVERLOAD                  0x4002
#define SA_CTL_EVENT_POWER_SUPPLY_FAILURE                   0x4003
#define SA_CTL_EVENT_FAN_FAILURE_STATE_CHANGED              0x4004
#define SA_CTL_EVENT_ADJUSTMENT_FINISHED                    0x4010
#define SA_CTL_EVENT_ADJUSTMENT_STATE_CHANGED               0x4011
#define SA_CTL_EVENT_ADJUSTMENT_UPDATE                      0x4012
#define SA_CTL_EVENT_DIGITAL_INPUT_CHANGED                  0x4040
#define SA_CTL_EVENT_SM_DIGITAL_INPUT_CHANGED               0x4041

// device events (0x8000 - 0xbfff)
#define SA_CTL_EVENT_STREAM_FINISHED                        0x8000
#define SA_CTL_EVENT_STREAM_READY                           0x8001
#define SA_CTL_EVENT_STREAM_TRIGGERED                       0x8002
#define SA_CTL_EVENT_CMD_GROUP_TRIGGERED                    0x8010
#define SA_CTL_EVENT_HM_STATE_CHANGED                       0x8020
#define SA_CTL_EVENT_EMERGENCY_STOP_TRIGGERED               0x8030
#define SA_CTL_EVENT_EXT_INPUT_TRIGGERED                    0x8040
#define SA_CTL_EVENT_BUS_RESYNC_TRIGGERED                   0x8050
#define SA_CTL_EVENT_UNKNOWN_COMMAND_RECEIVED               0x8051
// api events (0xf000 - 0xffff)
#define SA_CTL_EVENT_REQUEST_READY                          0xf000
#define SA_CTL_EVENT_CONNECTION_LOST                        0xf001

// @endconstants EventType

/**********************************************************/
/* EVENT PARAMETERS                                       */
/**********************************************************/

// @constants EventParameter

#define SA_CTL_EVENT_PARAM_DETACHED                         0x00000000
#define SA_CTL_EVENT_PARAM_ATTACHED                         0x00000001
#define SA_CTL_EVENT_PARAM_DISABLED                         0x00000000
#define SA_CTL_EVENT_PARAM_ENABLED                          0x00000001

#define SA_CTL_EVENT_REQ_READY_TYPE_READ                    0x00
#define SA_CTL_EVENT_REQ_READY_TYPE_WRITE                   0x01

// event parameter decoding
#define SA_CTL_EVENT_PARAM_RESULT_MASK                      0x0000ffff
#define SA_CTL_EVENT_PARAM_INDEX_MASK                       0x00ff0000
#define SA_CTL_EVENT_PARAM_HANDLE_MASK                      0xff000000
#define SA_CTL_EVENT_PARAM_RESULT(param)                    (((param) & SA_CTL_EVENT_PARAM_RESULT_MASK) >> 0)
#define SA_CTL_EVENT_PARAM_INDEX(param)                     (((param) & SA_CTL_EVENT_PARAM_INDEX_MASK) >> 16)
#define SA_CTL_EVENT_PARAM_HANDLE(param)                    (((param) & SA_CTL_EVENT_PARAM_HANDLE_MASK) >> 24)

#define SA_CTL_EVENT_REQ_READY_ID_MASK                      0x00000000000000ff
#define SA_CTL_EVENT_REQ_READY_TYPE_MASK                    0x000000000000ff00
#define SA_CTL_EVENT_REQ_READY_DATA_TYPE_MASK               0x0000000000ff0000
#define SA_CTL_EVENT_REQ_READY_ARRAY_SIZE_MASK              0x00000000ff000000
#define SA_CTL_EVENT_REQ_READY_PROPERTY_KEY_MASK            0xffffffff00000000
#define SA_CTL_EVENT_REQ_READY_ID(param)                    (((param) & SA_CTL_EVENT_REQ_READY_ID_MASK) >> 0)
#define SA_CTL_EVENT_REQ_READY_TYPE(param)                  (((param) & SA_CTL_EVENT_REQ_READY_TYPE_MASK) >> 8)
#define SA_CTL_EVENT_REQ_READY_DATA_TYPE(param)             (((param) & SA_CTL_EVENT_REQ_READY_DATA_TYPE_MASK) >> 16)
#define SA_CTL_EVENT_REQ_READY_ARRAY_SIZE(param)            (((param) & SA_CTL_EVENT_REQ_READY_ARRAY_SIZE_MASK) >> 24)
#define SA_CTL_EVENT_REQ_READY_PROPERTY_KEY(param)          (((param) & SA_CTL_EVENT_REQ_READY_PROPERTY_KEY_MASK) >> 32)

// @endconstants EventParameter

 /*********************************************************/
 /* ERROR CODES                                           */
 /*********************************************************/

// @constants ErrorCode

#define SA_CTL_ERROR_NONE                                   0x0000
#define SA_CTL_ERROR_UNKNOWN_COMMAND                        0x0001
#define SA_CTL_ERROR_INVALID_PACKET_SIZE                    0x0002
#define SA_CTL_ERROR_TIMEOUT                                0x0004
#define SA_CTL_ERROR_INVALID_PROTOCOL                       0x0005
#define SA_CTL_ERROR_BUFFER_UNDERFLOW                       0x000c
#define SA_CTL_ERROR_BUFFER_OVERFLOW                        0x000d
#define SA_CTL_ERROR_INVALID_FRAME_SIZE                     0x000e
#define SA_CTL_ERROR_INVALID_PACKET                         0x0010
#define SA_CTL_ERROR_INVALID_KEY                            0x0012
#define SA_CTL_ERROR_INVALID_PARAMETER                      0x0013
#define SA_CTL_ERROR_INVALID_DATA_TYPE                      0x0016
#define SA_CTL_ERROR_INVALID_DATA                           0x0017
#define SA_CTL_ERROR_HANDLE_LIMIT_REACHED                   0x0018
#define SA_CTL_ERROR_ABORTED                                0x0019

#define SA_CTL_ERROR_INVALID_DEVICE_INDEX                   0x0020
#define SA_CTL_ERROR_INVALID_MODULE_INDEX                   0x0021
#define SA_CTL_ERROR_INVALID_CHANNEL_INDEX                  0x0022

#define SA_CTL_ERROR_PERMISSION_DENIED                      0x0023
#define SA_CTL_ERROR_COMMAND_NOT_GROUPABLE                  0x0024
#define SA_CTL_ERROR_MOVEMENT_LOCKED                        0x0025
#define SA_CTL_ERROR_SYNC_FAILED                            0x0026
#define SA_CTL_ERROR_INVALID_ARRAY_SIZE                     0x0027
#define SA_CTL_ERROR_OVERRANGE                              0x0028
#define SA_CTL_ERROR_INVALID_CONFIGURATION                  0x0029
#define SA_CTL_ERROR_INVALID_GROUP_HANDLE                   0x002a

#define SA_CTL_ERROR_NO_HM_PRESENT                          0x0100
#define SA_CTL_ERROR_NO_IOM_PRESENT                         0x0101
#define SA_CTL_ERROR_NO_SM_PRESENT                          0x0102
#define SA_CTL_ERROR_NO_SENSOR_PRESENT                      0x0103
#define SA_CTL_ERROR_SENSOR_DISABLED                        0x0104
#define SA_CTL_ERROR_POWER_SUPPLY_DISABLED                  0x0105
#define SA_CTL_ERROR_AMPLIFIER_DISABLED                     0x0106
#define SA_CTL_ERROR_INVALID_SENSOR_MODE                    0x0107
#define SA_CTL_ERROR_INVALID_ACTUATOR_MODE                  0x0108
#define SA_CTL_ERROR_INVALID_INPUT_TRIG_MODE                0x0109
#define SA_CTL_ERROR_INVALID_CONTROL_OPTIONS                0x010a
#define SA_CTL_ERROR_INVALID_REFERENCE_TYPE                 0x010b
#define SA_CTL_ERROR_INVALID_ADJUSTMENT_STATE               0x010c
#define SA_CTL_ERROR_INVALID_INFO_TYPE                      0x010d
#define SA_CTL_ERROR_NO_FULL_ACCESS                         0x010e
#define SA_CTL_ERROR_ADJUSTMENT_FAILED                      0x010f
#define SA_CTL_ERROR_MOVEMENT_OVERRIDDEN                    0x0110
#define SA_CTL_ERROR_NOT_CALIBRATED                         0x0111
#define SA_CTL_ERROR_NOT_REFERENCED                         0x0112
#define SA_CTL_ERROR_NOT_ADJUSTED                           0x0113
#define SA_CTL_ERROR_SENSOR_TYPE_NOT_SUPPORTED              0x0114
#define SA_CTL_ERROR_CONTROL_LOOP_INPUT_DISABLED            0x0115
#define SA_CTL_ERROR_INVALID_CONTROL_LOOP_INPUT             0x0116
#define SA_CTL_ERROR_UNEXPECTED_SENSOR_DATA                 0x0117
#define SA_CTL_ERROR_NOT_PHASED                             0x0118
#define SA_CTL_ERROR_POSITIONER_FAULT                       0x0119
#define SA_CTL_ERROR_DRIVER_FAULT                           0x011a
#define SA_CTL_ERROR_POSITIONER_TYPE_NOT_SUPPORTED          0x011b
#define SA_CTL_ERROR_POSITIONER_TYPE_NOT_IDENTIFIED         0x011c
#define SA_CTL_ERROR_POSITIONER_TYPE_NOT_WRITEABLE          0x011e
#define SA_CTL_ERROR_INVALID_ACTUATOR_TYPE                  0x0121
#define SA_CTL_ERROR_NO_COMMUTATION_SENSOR_PRESENT          0x0122
#define SA_CTL_ERROR_AMPLIFIER_LOCKED                       0x0123
#define SA_CTL_ERROR_WRITE_ACCESS_LOCKED                    0x0124
#define SA_CTL_ERROR_UNEXPECTED_SCAN_RANGE                  0x0125

#define SA_CTL_ERROR_BUSY_MOVING                            0x0150
#define SA_CTL_ERROR_BUSY_CALIBRATING                       0x0151
#define SA_CTL_ERROR_BUSY_REFERENCING                       0x0152
#define SA_CTL_ERROR_BUSY_ADJUSTING                         0x0153
#define SA_CTL_ERROR_BUSY_CHANGING_AMP_STATE                0x0155

#define SA_CTL_ERROR_END_STOP_REACHED                       0x0200
#define SA_CTL_ERROR_FOLLOWING_ERR_LIMIT                    0x0201
#define SA_CTL_ERROR_RANGE_LIMIT_REACHED                    0x0202
#define SA_CTL_ERROR_POSITIONER_OVERLOAD                    0x0203
#define SA_CTL_ERROR_POWER_SUPPLY_FAILURE                   0x0205
#define SA_CTL_ERROR_OVER_TEMPERATURE                       0x0206
#define SA_CTL_ERROR_POWER_SUPPLY_OVERLOAD                  0x0208
#define SA_CTL_ERROR_CONTROL_LOOP_UNSTABLE                  0x0209

#define SA_CTL_ERROR_INVALID_STREAM_HANDLE                  0x0300
#define SA_CTL_ERROR_INVALID_STREAM_CONFIGURATION           0x0301
#define SA_CTL_ERROR_INSUFFICIENT_FRAMES                    0x0302
#define SA_CTL_ERROR_BUSY_STREAMING                         0x0303

#define SA_CTL_ERROR_HM_INVALID_SLOT_INDEX                  0x0400
#define SA_CTL_ERROR_HM_INVALID_CHANNEL_INDEX               0x0401
#define SA_CTL_ERROR_HM_INVALID_GROUP_INDEX                 0x0402
#define SA_CTL_ERROR_HM_INVALID_CH_GRP_INDEX                0x0403

#define SA_CTL_ERROR_INTERNAL_COMMUNICATION                 0x0500
#define SA_CTL_ERROR_EEPROM_BUFFER_OVERFLOW                 0x0501

#define SA_CTL_ERROR_FEATURE_NOT_SUPPORTED                  0x7ffd
#define SA_CTL_ERROR_FEATURE_NOT_IMPLEMENTED                0x7ffe

// api error codes
#define SA_CTL_ERROR_DEVICE_LIMIT_REACHED                   0xf000
#define SA_CTL_ERROR_INVALID_LOCATOR                        0xf001
#define SA_CTL_ERROR_INITIALIZATION_FAILED                  0xf002
#define SA_CTL_ERROR_NOT_INITIALIZED                        0xf003
#define SA_CTL_ERROR_COMMUNICATION_FAILED                   0xf004
#define SA_CTL_ERROR_INVALID_QUERYBUFFER_SIZE               0xf006
#define SA_CTL_ERROR_INVALID_DEVICE_HANDLE                  0xf007
#define SA_CTL_ERROR_INVALID_TRANSMIT_HANDLE                0xf008
#define SA_CTL_ERROR_UNEXPECTED_PACKET_RECEIVED             0xf00f
#define SA_CTL_ERROR_CANCELED                               0xf010
#define SA_CTL_ERROR_DRIVER_FAILED                          0xf013
#define SA_CTL_ERROR_BUFFER_LIMIT_REACHED                   0xf016
#define SA_CTL_ERROR_INVALID_PROTOCOL_VERSION               0xf017
#define SA_CTL_ERROR_DEVICE_RESET_FAILED                    0xf018
#define SA_CTL_ERROR_BUFFER_EMPTY                           0xf019
#define SA_CTL_ERROR_DEVICE_NOT_FOUND                       0xf01a
#define SA_CTL_ERROR_THREAD_LIMIT_REACHED                   0xf01b
#define SA_CTL_ERROR_NO_APPLICATION                         0xf01c

// @endconstants ErrorCode

/**********************************************************/
/* DATA TYPES                                             */
/**********************************************************/

// @constants DataType

#define SA_CTL_DTYPE_UINT16                                 0x03
#define SA_CTL_DTYPE_INT32                                  0x06
#define SA_CTL_DTYPE_INT64                                  0x0e
#define SA_CTL_DTYPE_FLOAT32                                0x10
#define SA_CTL_DTYPE_FLOAT64                                0x11
#define SA_CTL_DTYPE_STRING                                 0x12
#define SA_CTL_DTYPE_NONE                                   0xff

// @endconstants DataType

/**********************************************************/
/* BASE UNIT TYPES                                        */
/**********************************************************/

// @constants BaseUnit

#define SA_CTL_UNIT_NONE                                    0x00000000
#define SA_CTL_UNIT_PERCENT                                 0x00000001
#define SA_CTL_UNIT_METER                                   0x00000002
#define SA_CTL_UNIT_DEGREE                                  0x00000003
#define SA_CTL_UNIT_SECOND                                  0x00000004
#define SA_CTL_UNIT_HERTZ                                   0x00000005

// @endconstants BaseUnit

/**********************************************************/
/* PROPERTY KEYS                                          */
/**********************************************************/

// @constants Property

// device
#define SA_CTL_PKEY_NUMBER_OF_CHANNELS                      0x020F0017
#define SA_CTL_PKEY_NUMBER_OF_BUS_MODULES                   0x020F0016
#define SA_CTL_PKEY_INTERFACE_TYPE                          0x020F0066
#define SA_CTL_PKEY_DEVICE_STATE                            0x020F000F
#define SA_CTL_PKEY_DEVICE_SERIAL_NUMBER                    0x020F005E
#define SA_CTL_PKEY_DEVICE_NAME                             0x020F003D
#define SA_CTL_PKEY_EMERGENCY_STOP_MODE                     0x020F0088
#define SA_CTL_PKEY_DEFAULT_EMERGENCY_STOP_MODE             0x020F0116
#define SA_CTL_PKEY_NETWORK_DISCOVER_MODE                   0x020F0159
#define SA_CTL_PKEY_NETWORK_DHCP_TIMEOUT                    0x020F015C
#define SA_CTL_PKEY_MIN_FAN_LEVEL                           0x020F00DB
// module
#define SA_CTL_PKEY_POWER_SUPPLY_ENABLED                    0x02030010
#define SA_CTL_PKEY_NUMBER_OF_BUS_MODULE_CHANNELS           0x02030017
#define SA_CTL_PKEY_MODULE_TYPE                             0x02030066
#define SA_CTL_PKEY_MODULE_STATE                            0x0203000F
// positioner
#define SA_CTL_PKEY_STARTUP_OPTIONS                         0x0A02005D
#define SA_CTL_PKEY_AMPLIFIER_ENABLED                       0x0302000D
#define SA_CTL_PKEY_AMPLIFIER_MODE                          0x030200BF
#define SA_CTL_PKEY_POSITIONER_CONTROL_OPTIONS              0x0302005D
#define SA_CTL_PKEY_ACTUATOR_MODE                           0x03020019
#define SA_CTL_PKEY_CONTROL_LOOP_INPUT                      0x03020018
#define SA_CTL_PKEY_SENSOR_INPUT_SELECT                     0x0302009D
#define SA_CTL_PKEY_POSITIONER_TYPE                         0x0302003C
#define SA_CTL_PKEY_POSITIONER_TYPE_NAME                    0x0302003D
#define SA_CTL_PKEY_MOVE_MODE                               0x03050087
#define SA_CTL_PKEY_CHANNEL_TYPE                            0x02020066
#define SA_CTL_PKEY_CHANNEL_STATE                           0x0305000F
#define SA_CTL_PKEY_POSITION                                0x0305001D
#define SA_CTL_PKEY_TARGET_POSITION                         0x0305001E
#define SA_CTL_PKEY_SCAN_POSITION                           0x0305001F
#define SA_CTL_PKEY_SCAN_VELOCITY                           0x0305002A
#define SA_CTL_PKEY_HOLD_TIME                               0x03050028
#define SA_CTL_PKEY_MOVE_VELOCITY                           0x03050029
#define SA_CTL_PKEY_MOVE_ACCELERATION                       0x0305002B
#define SA_CTL_PKEY_MAX_CL_FREQUENCY                        0x0305002F
#define SA_CTL_PKEY_DEFAULT_MAX_CL_FREQUENCY                0x03050057
#define SA_CTL_PKEY_STEP_FREQUENCY                          0x0305002E
#define SA_CTL_PKEY_STEP_AMPLITUDE                          0x03050030
#define SA_CTL_PKEY_FOLLOWING_ERROR_LIMIT                   0x03050055
#define SA_CTL_PKEY_FOLLOWING_ERROR                         0x03020055
#define SA_CTL_PKEY_FOLLOWING_ERROR_MAX                     0x05020055
#define SA_CTL_PKEY_BROADCAST_STOP_OPTIONS                  0x0305005D
#define SA_CTL_PKEY_SENSOR_POWER_MODE                       0x03080019
#define SA_CTL_PKEY_SENSOR_POWER_SAVE_DELAY                 0x03080054
#define SA_CTL_PKEY_POSITION_MEAN_SHIFT                     0x03090022
#define SA_CTL_PKEY_SAFE_DIRECTION                          0x03090027
#define SA_CTL_PKEY_CL_INPUT_SENSOR_VALUE                   0x0302001D
#define SA_CTL_PKEY_CL_INPUT_AUX_VALUE                      0x030200B2
#define SA_CTL_PKEY_TARGET_TO_ZERO_VOLTAGE_HOLD_TH          0x030200B9
#define SA_CTL_PKEY_HOLDING_GAIN_REDUCTION                  0x0302004E
#define SA_CTL_PKEY_CH_EMERGENCY_STOP_MODE                  0x02020088
#define SA_CTL_PKEY_IN_POSITION_THRESHOLD                   0x03050058
#define SA_CTL_PKEY_IN_POSITION_DELAY                       0x03050054
#define SA_CTL_PKEY_MOTOR_LOAD_PROTECTION_THRESHOLD         0x03020115
#define SA_CTL_PKEY_BRAKE_OFF_DELAY                         0x03050117
#define SA_CTL_PKEY_BRAKE_ON_DELAY                          0x03050118
#define SA_CTL_PKEY_RESONANCE_FREQUENCY_1                   0x030900E0
#define SA_CTL_PKEY_RESONANCE_FREQUENCY_2                   0x030900E1

// scale
#define SA_CTL_PKEY_LOGICAL_SCALE_OFFSET                    0x02040024
#define SA_CTL_PKEY_LOGICAL_SCALE_INVERSION                 0x02040025
#define SA_CTL_PKEY_RANGE_LIMIT_MIN                         0x02040020
#define SA_CTL_PKEY_RANGE_LIMIT_MAX                         0x02040021
#define SA_CTL_PKEY_DEFAULT_RANGE_LIMIT_MIN                 0x020400C0
#define SA_CTL_PKEY_DEFAULT_RANGE_LIMIT_MAX                 0x020400C1
// calibration
#define SA_CTL_PKEY_CALIBRATION_OPTIONS                     0x0306005D
#define SA_CTL_PKEY_SIGNAL_CORRECTION_OPTIONS               0x0306001C
// referencing
#define SA_CTL_PKEY_REFERENCING_OPTIONS                     0x0307005D
#define SA_CTL_PKEY_DIST_CODE_INVERTED                      0x0307000E
#define SA_CTL_PKEY_DISTANCE_TO_REF_MARK                    0x030700A2
// tuning and customizing
#define SA_CTL_PKEY_POS_MOVEMENT_TYPE                       0x0309003F
#define SA_CTL_PKEY_POS_IS_CUSTOM_TYPE                      0x03090041
#define SA_CTL_PKEY_POS_BASE_UNIT                           0x03090042
#define SA_CTL_PKEY_POS_BASE_RESOLUTION                     0x03090043
#define SA_CTL_PKEY_POS_HEAD_TYPE                           0x0309008E
#define SA_CTL_PKEY_POS_REF_TYPE                            0x03090048
#define SA_CTL_PKEY_POS_P_GAIN                              0x0309004B
#define SA_CTL_PKEY_POS_I_GAIN                              0x0309004C
#define SA_CTL_PKEY_POS_D_GAIN                              0x0309004D
#define SA_CTL_PKEY_POS_PID_SHIFT                           0x0309004E
#define SA_CTL_PKEY_POS_ANTI_WINDUP                         0x0309004F
#define SA_CTL_PKEY_POS_ESD_DIST_TH                         0x03090050
#define SA_CTL_PKEY_POS_ESD_COUNTER_TH                      0x03090051
#define SA_CTL_PKEY_POS_TARGET_REACHED_TH                   0x03090052
#define SA_CTL_PKEY_POS_TARGET_HOLD_TH                      0x03090053
#define SA_CTL_PKEY_POS_SAVE                                0x0309000A
#define SA_CTL_PKEY_POS_WRITE_PROTECTION                    0x0309000D
// streaming
#define SA_CTL_PKEY_STREAM_BASE_RATE                        0x040F002C
#define SA_CTL_PKEY_STREAM_EXT_SYNC_RATE                    0x040F002D
#define SA_CTL_PKEY_STREAM_OPTIONS                          0x040F005D
#define SA_CTL_PKEY_STREAM_LOAD_MAX                         0x040F0301
// diagnostic
#define SA_CTL_PKEY_CHANNEL_ERROR                           0x0502007A
#define SA_CTL_PKEY_CHANNEL_TEMPERATURE                     0x05020034
#define SA_CTL_PKEY_BUS_MODULE_TEMPERATURE                  0x05030034
#define SA_CTL_PKEY_POSITIONER_FAULT_REASON                 0x05020113
#define SA_CTL_PKEY_MOTOR_LOAD                              0x05020115
#define SA_CTL_PKEY_MOTOR_CURRENT                           0x05020110
#define SA_CTL_PKEY_DIAG_CLOSED_LOOP_FREQUENCY_AVG          0x0502002e
#define SA_CTL_PKEY_DIAG_CLOSED_LOOP_FREQUENCY_MAX          0x0502002f
#define SA_CTL_PKEY_DIAG_CLF_MEASURE_TIME_BASE              0x050200c6
// io module
#define SA_CTL_PKEY_IO_MODULE_OPTIONS                       0x0603005D
#define SA_CTL_PKEY_IO_MODULE_VOLTAGE                       0x06030031
#define SA_CTL_PKEY_IO_MODULE_ANALOG_INPUT_RANGE            0x060300A0
// sensor module
#define SA_CTL_PKEY_SENSOR_MODULE_OPTIONS                   0x080B005D
// auxiliary
#define SA_CTL_PKEY_AUX_POSITIONER_TYPE                     0x0802003C
#define SA_CTL_PKEY_AUX_POSITIONER_TYPE_NAME                0x0802003D
#define SA_CTL_PKEY_AUX_INPUT_SELECT                        0x08020018
#define SA_CTL_PKEY_AUX_IO_MODULE_INPUT_INDEX               0x081100AA
#define SA_CTL_PKEY_AUX_SENSOR_MODULE_INPUT_INDEX           0x080B00AA
#define SA_CTL_PKEY_AUX_IO_MODULE_INPUT0_VALUE              0x08110000
#define SA_CTL_PKEY_AUX_IO_MODULE_INPUT1_VALUE              0x08110001
#define SA_CTL_PKEY_AUX_SENSOR_MODULE_INPUT0_VALUE          0x080B0000
#define SA_CTL_PKEY_AUX_SENSOR_MODULE_INPUT1_VALUE          0x080B0001
#define SA_CTL_PKEY_AUX_DIRECTION_INVERSION                 0x0809000E
#define SA_CTL_PKEY_AUX_DIGITAL_INPUT_VALUE                 0x080300AD
#define SA_CTL_PKEY_AUX_SM_DIGITAL_INPUT_VALUE              0x080B00AD
#define SA_CTL_PKEY_AUX_DIGITAL_OUTPUT_VALUE                0x080300AE
#define SA_CTL_PKEY_AUX_DIGITAL_OUTPUT_SET                  0x080300B0
#define SA_CTL_PKEY_AUX_DIGITAL_OUTPUT_CLEAR                0x080300B1
#define SA_CTL_PKEY_AUX_ANALOG_OUTPUT_VALUE0                0x08030000
#define SA_CTL_PKEY_AUX_ANALOG_OUTPUT_VALUE1                0x08030001
#define SA_CTL_PKEY_AUX_CORRECTION_MAX                      0x080200D8
// threshold detector
#define SA_CTL_PKEY_THD_INPUT_SELECT                        0x09020018
#define SA_CTL_PKEY_THD_IO_MODULE_INPUT_INDEX               0x091100AA
#define SA_CTL_PKEY_THD_SENSOR_MODULE_INPUT_INDEX           0x090B00AA
#define SA_CTL_PKEY_THD_THRESHOLD_HIGH                      0x090200B4
#define SA_CTL_PKEY_THD_THRESHOLD_LOW                       0x090200B5
#define SA_CTL_PKEY_THD_INVERSION                           0x0902000E
// input trigger
#define SA_CTL_PKEY_DEV_INPUT_TRIG_SELECT                   0x060D009D
#define SA_CTL_PKEY_DEV_INPUT_TRIG_MODE                     0x060D0087
#define SA_CTL_PKEY_DEV_INPUT_TRIG_CONDITION                0x060D005A
#define SA_CTL_PKEY_DEV_INPUT_TRIG_DEBOUNCE                 0x060D0058
#define SA_CTL_PKEY_CH_INPUT_TRIG_SELECT                    0x0615009D
#define SA_CTL_PKEY_CH_INPUT_TRIG_MODE                      0x06150087
#define SA_CTL_PKEY_CH_INPUT_TRIG_CONDITION                 0x0615005A
// output trigger
#define SA_CTL_PKEY_CH_OUTPUT_TRIG_MODE                     0x060E0087
#define SA_CTL_PKEY_CH_OUTPUT_TRIG_POLARITY                 0x060E005B
#define SA_CTL_PKEY_CH_OUTPUT_TRIG_PULSE_WIDTH              0x060E005C
#define SA_CTL_PKEY_CH_POS_COMP_START_THRESHOLD             0x060E0058
#define SA_CTL_PKEY_CH_POS_COMP_INCREMENT                   0x060E0059
#define SA_CTL_PKEY_CH_POS_COMP_DIRECTION                   0x060E0026
#define SA_CTL_PKEY_CH_POS_COMP_LIMIT_MIN                   0x060E0020
#define SA_CTL_PKEY_CH_POS_COMP_LIMIT_MAX                   0x060E0021
// hand control module
#define SA_CTL_PKEY_HM_STATE                                0x020C000F
#define SA_CTL_PKEY_HM_LOCK_OPTIONS                         0x020C0083
#define SA_CTL_PKEY_HM_DEFAULT_LOCK_OPTIONS                 0x020C0084
// api
#define SA_CTL_PKEY_API_EVENT_NOTIFICATION_OPTIONS          0xF010005D
#define SA_CTL_PKEY_EVENT_NOTIFICATION_OPTIONS              0xF010005D  // deprecated
#define SA_CTL_PKEY_API_AUTO_RECONNECT                      0xF01000A1
#define SA_CTL_PKEY_AUTO_RECONNECT                          0xF01000A1  // deprecated

// @endconstants Property

/**********************************************************/
/* STATE BITS                                             */
/**********************************************************/

// @constants DeviceState

// device states
#define SA_CTL_DEV_STATE_BIT_HM_PRESENT                     0x00000001
#define SA_CTL_DEV_STATE_BIT_MOVEMENT_LOCKED                0x00000002
#define SA_CTL_DEV_STATE_BIT_AMPLIFIER_LOCKED               0x00000004
#define SA_CTL_DEV_STATE_BIT_IO_MODULE_INPUT                0x00000008
#define SA_CTL_DEV_STATE_BIT_GLOBAL_INPUT                   0x00000010
#define SA_CTL_DEV_STATE_BIT_INTERNAL_COMM_FAILURE          0x00000100
#define SA_CTL_DEV_STATE_BIT_IS_STREAMING                   0x00001000
#define SA_CTL_DEV_STATE_BIT_EEPROM_BUSY                    0x00010000

// @endconstants DeviceState

// @constants ModuleState

#define SA_CTL_MOD_STATE_BIT_SM_PRESENT                     0x00000001
#define SA_CTL_MOD_STATE_BIT_BOOSTER_PRESENT                0x00000002
#define SA_CTL_MOD_STATE_BIT_ADJUSTMENT_ACTIVE              0x00000004
#define SA_CTL_MOD_STATE_BIT_IOM_PRESENT                    0x00000008
#define SA_CTL_MOD_STATE_BIT_INTERNAL_COMM_FAILURE          0x00000100
#define SA_CTL_MOD_STATE_BIT_FAN_FAILURE                    0x00000800
#define SA_CTL_MOD_STATE_BIT_POWER_SUPPLY_FAILURE           0x00001000
#define SA_CTL_MOD_STATE_BIT_HIGH_VOLTAGE_FAILURE           0x00001000  // deprecated
#define SA_CTL_MOD_STATE_BIT_POWER_SUPPLY_OVERLOAD          0x00002000
#define SA_CTL_MOD_STATE_BIT_HIGH_VOLTAGE_OVERLOAD          0x00002000  // deprecated
#define SA_CTL_MOD_STATE_BIT_OVER_TEMPERATURE               0x00004000
#define SA_CTL_MOD_STATE_BIT_EEPROM_BUSY                    0x00010000

// @endconstants ModuleState

// @constants ChannelState

#define SA_CTL_CH_STATE_BIT_ACTIVELY_MOVING                 0x00000001
#define SA_CTL_CH_STATE_BIT_CLOSED_LOOP_ACTIVE              0x00000002
#define SA_CTL_CH_STATE_BIT_CALIBRATING                     0x00000004
#define SA_CTL_CH_STATE_BIT_REFERENCING                     0x00000008
#define SA_CTL_CH_STATE_BIT_MOVE_DELAYED                    0x00000010
#define SA_CTL_CH_STATE_BIT_SENSOR_PRESENT                  0x00000020
#define SA_CTL_CH_STATE_BIT_IS_CALIBRATED                   0x00000040
#define SA_CTL_CH_STATE_BIT_IS_REFERENCED                   0x00000080
#define SA_CTL_CH_STATE_BIT_END_STOP_REACHED                0x00000100
#define SA_CTL_CH_STATE_BIT_RANGE_LIMIT_REACHED             0x00000200
#define SA_CTL_CH_STATE_BIT_FOLLOWING_LIMIT_REACHED         0x00000400
#define SA_CTL_CH_STATE_BIT_MOVEMENT_FAILED                 0x00000800
#define SA_CTL_CH_STATE_BIT_IS_STREAMING                    0x00001000
#define SA_CTL_CH_STATE_BIT_POSITIONER_OVERLOAD             0x00002000
#define SA_CTL_CH_STATE_BIT_OVER_TEMPERATURE                0x00004000
#define SA_CTL_CH_STATE_BIT_REFERENCE_MARK                  0x00008000
#define SA_CTL_CH_STATE_BIT_IS_PHASED                       0x00010000
#define SA_CTL_CH_STATE_BIT_POSITIONER_FAULT                0x00020000
#define SA_CTL_CH_STATE_BIT_AMPLIFIER_ENABLED               0x00040000
#define SA_CTL_CH_STATE_BIT_IN_POSITION                     0x00080000
#define SA_CTL_CH_STATE_BIT_BRAKE_ENABLED                   0x00100000

// @endconstants ChannelState

// @constants HMState

#define SA_CTL_HM_STATE_BIT_INTERNAL_COMM_FAILURE           0x00000100
#define SA_CTL_HM_STATE_BIT_IS_INTERNAL                     0x00000200
#define SA_CTL_HM_STATE_BIT_EEPROM_BUSY                     0x00010000

// @endconstants HMState

/**********************************************************/
/* MOVEMENT MODES                                         */
/**********************************************************/

// @constants MoveMode

#define SA_CTL_MOVE_MODE_CL_ABSOLUTE                        0
#define SA_CTL_MOVE_MODE_CL_RELATIVE                        1
#define SA_CTL_MOVE_MODE_SCAN_ABSOLUTE                      2
#define SA_CTL_MOVE_MODE_SCAN_RELATIVE                      3
#define SA_CTL_MOVE_MODE_STEP                               4

// @endconstants MoveMode

/**********************************************************/
/* ACTUATOR MODES                                         */
/**********************************************************/

// @constants ActuatorMode

#define SA_CTL_ACTUATOR_MODE_NORMAL                         0
#define SA_CTL_ACTUATOR_MODE_QUIET                          1
#define SA_CTL_ACTUATOR_MODE_LOW_VIBRATION                  2

// @endconstants ActuatorMode

/**********************************************************/
/* CONTROL LOOP INPUT                                     */
/**********************************************************/

// @constants ControlLoopInput

#define SA_CTL_CONTROL_LOOP_INPUT_DISABLED                  0
#define SA_CTL_CONTROL_LOOP_INPUT_SENSOR                    1
#define SA_CTL_CONTROL_LOOP_INPUT_POSITION                  1  // deprecated
#define SA_CTL_CONTROL_LOOP_INPUT_AUX_IN                    2

// @endconstants ControlLoopInput

/**********************************************************/
/* SENSOR INPUT SELECT                                    */
/**********************************************************/

// @constants SensorInputSelect

#define SA_CTL_SENSOR_INPUT_SELECT_POSITION                 0
#define SA_CTL_SENSOR_INPUT_SELECT_CALC_SYS                 1
#define SA_CTL_SENSOR_INPUT_SELECT_POS_PLUS_AUX_IN          2

// @endconstants SensorInputSelect

/**********************************************************/
/* AUX INPUT SELECT                                       */
/**********************************************************/

// @constants AuxInputSelect

#define SA_CTL_AUX_INPUT_SELECT_IO_MODULE                   0
#define SA_CTL_AUX_INPUT_SELECT_SENSOR_MODULE               1

// @endconstants AuxInputSelect

/**********************************************************/
/* I/O MODULE DIGITAL INPUT                               */
/**********************************************************/

// @constants IODigitalInput

#define SA_CTL_IO_DIGITAL_INPUT_BIT_GP_IN_1                 0x00000001
#define SA_CTL_IO_DIGITAL_INPUT_BIT_GP_IN_2                 0x00000002
#define SA_CTL_IO_DIGITAL_INPUT_BIT_GP_IN_3                 0x00000004
#define SA_CTL_IO_DIGITAL_INPUT_BIT_GP_IN_4                 0x00000008

// @endconstants IODigitalInput

/**********************************************************/
/* I/O MODULE DIGITAL OUTPUT                              */
/**********************************************************/

// @constants IODigitalOutput

#define SA_CTL_IO_DIGITAL_OUTPUT_BIT_GP_OUT_1               0x00000001
#define SA_CTL_IO_DIGITAL_OUTPUT_BIT_GP_OUT_2               0x00000002
#define SA_CTL_IO_DIGITAL_OUTPUT_BIT_GP_OUT_3               0x00000004
#define SA_CTL_IO_DIGITAL_OUTPUT_BIT_GP_OUT_4               0x00000008

// @endconstants IODigitalOutput

/**********************************************************/
/* SENSOR MODULE DIGITAL INPUT                            */
/**********************************************************/

// @constants SMDigitalInput

#define SA_CTL_SM_DIGITAL_INPUT_BIT_POWER_SUPPLY            0x00000001
#define SA_CTL_SM_DIGITAL_INPUT_BIT_PRESSURE_SENSOR         0x00000002
#define SA_CTL_SM_DIGITAL_INPUT_BIT_ES_1                    0x00000004
#define SA_CTL_SM_DIGITAL_INPUT_BIT_ES_2                    0x00000008
#define SA_CTL_SM_DIGITAL_INPUT_BIT_GP_IN_1                 0x00000010
#define SA_CTL_SM_DIGITAL_INPUT_BIT_GP_IN_2                 0x00000020
#define SA_CTL_SM_DIGITAL_INPUT_BIT_GP_IN_3                 0x00000040
#define SA_CTL_SM_DIGITAL_INPUT_BIT_SYNC                    0x00000080

// @endconstants SMDigitalInput

/**********************************************************/
/* THRESHOLD DETECTOR INPUT SELECT                        */
/**********************************************************/

// @constants THDInputSelect

#define SA_CTL_THD_INPUT_SELECT_IO_MODULE                   0
#define SA_CTL_THD_INPUT_SELECT_SENSOR_MODULE               1

// @endconstants THDInputSelect

/**********************************************************/
/* EMERGENCY STOP MODES                                   */
/**********************************************************/

// @constants EmergencyStopMode

#define SA_CTL_EMERGENCY_STOP_MODE_NORMAL                   0
#define SA_CTL_EMERGENCY_STOP_MODE_RESTRICTED               1
#define SA_CTL_EMERGENCY_STOP_MODE_AUTO_RELEASE             2

// @endconstants EmergencyStopMode

/**********************************************************/
/* COMMAND GROUP TRIGGER MODES                            */
/**********************************************************/

// @constants CmdGroupTriggerMode

#define SA_CTL_CMD_GROUP_TRIGGER_MODE_DIRECT                0
#define SA_CTL_CMD_GROUP_TRIGGER_MODE_EXTERNAL              1

// @endconstants CmdGroupTriggerMode

/**********************************************************/
/* STREAM TRIGGER MODES                                   */
/**********************************************************/

// @constants StreamTriggerMode

#define SA_CTL_STREAM_TRIGGER_MODE_DIRECT                   0
#define SA_CTL_STREAM_TRIGGER_MODE_EXTERNAL_ONCE            1
#define SA_CTL_STREAM_TRIGGER_MODE_EXTERNAL_SYNC            2
#define SA_CTL_STREAM_TRIGGER_MODE_EXTERNAL                 3

// @endconstants StreamTriggerMode

/**********************************************************/
/* STREAM OPTIONS                                         */
/**********************************************************/

// @constants StreamOption

#define SA_CTL_STREAM_OPT_BIT_INTERPOLATION_DIS             0x00000001

// @endconstants StreamOption

/**********************************************************/
/* STARTUP OPTIONS                                        */
/**********************************************************/

// @constants StartupOption

#define SA_CTL_STARTUP_OPT_BIT_AMPLIFIER_ENABLE             0x00000001

// @endconstants StartupOption

/**********************************************************/
/* POSITIONER CONTROL OPTIONS                             */
/**********************************************************/

// @constants PosControlOption

#define SA_CTL_POS_CTRL_OPT_BIT_ACC_REL_POS_DIS             0x00000001
#define SA_CTL_POS_CTRL_OPT_BIT_NO_SLIP                     0x00000002
#define SA_CTL_POS_CTRL_OPT_BIT_NO_SLIP_WHILE_HOLDING       0x00000004
#define SA_CTL_POS_CTRL_OPT_BIT_FORCED_SLIP_DIS             0x00000008
#define SA_CTL_POS_CTRL_OPT_BIT_STOP_ON_FOLLOWING_ERR       0x00000010
#define SA_CTL_POS_CTRL_OPT_BIT_TARGET_TO_ZERO_VOLTAGE      0x00000020
#define SA_CTL_POS_CTRL_OPT_BIT_CL_DIS_ON_FOLLOWING_ERR     0x00000040
#define SA_CTL_POS_CTRL_OPT_BIT_CL_DIS_ON_EMERGENCY_STOP    0x00000080
#define SA_CTL_POS_CTRL_OPT_BIT_IN_POSITION                 0x00000100
#define SA_CTL_POS_CTRL_OPT_BIT_COUPLED_BRAKE_CONTROL       0x00000200

// @endconstants PosControlOption

/**********************************************************/
/* CALIBRATION OPTIONS                                    */
/**********************************************************/

// @constants CalibrationOption

#define SA_CTL_CALIB_OPT_BIT_DIRECTION                      0x00000001
#define SA_CTL_CALIB_OPT_BIT_DIST_CODE_INV_DETECT           0x00000002
#define SA_CTL_CALIB_OPT_BIT_ASC_CALIBRATION                0x00000004
#define SA_CTL_CALIB_OPT_BIT_REF_MARK_TEST                  0x00000008
#define SA_CTL_CALIB_OPT_BIT_LIMITED_TRAVEL_RANGE           0x00000100

// @endconstants CalibrationOption

/**********************************************************/
/* REFERENCING OPTIONS                                    */
/**********************************************************/

// @constants ReferencingOption

#define SA_CTL_REF_OPT_BIT_START_DIR                        0x00000001
#define SA_CTL_REF_OPT_BIT_REVERSE_DIR                      0x00000002
#define SA_CTL_REF_OPT_BIT_AUTO_ZERO                        0x00000004
#define SA_CTL_REF_OPT_BIT_ABORT_ON_ENDSTOP                 0x00000008
#define SA_CTL_REF_OPT_BIT_CONTINUE_ON_REF_FOUND            0x00000010
#define SA_CTL_REF_OPT_BIT_STOP_ON_REF_FOUND                0x00000020
#define SA_CTL_REF_OPT_BIT_INVALIDATE                       0x01000000

// @endconstants ReferencingOption

/**********************************************************/
/* SENSOR POWER MODES                                     */
/**********************************************************/

// @constants SensorPowerMode

#define SA_CTL_SENSOR_MODE_DISABLED                         0
#define SA_CTL_SENSOR_MODE_ENABLED                          1
#define SA_CTL_SENSOR_MODE_POWER_SAVE                       2

// @endconstants SensorPowerMode

/**********************************************************/
/* BROADCAST STOP OPTIONS                                 */
/**********************************************************/

// @constants BroadcastStopOption

#define SA_CTL_STOP_OPT_BIT_END_STOP_REACHED                0x00000001
#define SA_CTL_STOP_OPT_BIT_RANGE_LIMIT_REACHED             0x00000002
#define SA_CTL_STOP_OPT_BIT_FOLLOWING_LIMIT_REACHED         0x00000004
#define SA_CTL_STOP_OPT_BIT_POSITIONER_OVERLOAD             0x00000008

// @endconstants BroadcastStopOption

/**********************************************************/
/* AMPLIFIER MODES                                        */
/**********************************************************/

// @constants AmplifierMode

#define SA_CTL_AMP_MODE_DEFAULT                             0
#define SA_CTL_AMP_MODE_POSITIONER_INTERLOCK                1

// @endconstants AmplifierMode

/**********************************************************/
/* INPUT/OUTPUT TRIGGER                                   */
/**********************************************************/

// @constants DeviceInputTriggerSelect

// device input trigger selectors
#define SA_CTL_DEV_INPUT_TRIG_SELECT_IO_MODULE              0
#define SA_CTL_DEV_INPUT_TRIG_SELECT_GLOBAL_INPUT           1

// @endconstants DeviceInputTriggerSelect

// @constants DeviceInputTriggerMode

// device input trigger modes
#define SA_CTL_DEV_INPUT_TRIG_MODE_DISABLED                 0
#define SA_CTL_DEV_INPUT_TRIG_MODE_EMERGENCY_STOP           1
#define SA_CTL_DEV_INPUT_TRIG_MODE_STREAM                   2
#define SA_CTL_DEV_INPUT_TRIG_MODE_CMD_GROUP                3
#define SA_CTL_DEV_INPUT_TRIG_MODE_EVENT                    4
#define SA_CTL_DEV_INPUT_TRIG_MODE_AMPLIFIER_LOCK           5

// @endconstants DeviceInputTriggerMode

// @constants ChannelInputTriggerSelect

// channel input trigger selectors
#define SA_CTL_CH_INPUT_TRIG_SELECT_IO_MODULE               0
#define SA_CTL_CH_INPUT_TRIG_SELECT_SENSOR_MODULE           1

// @endconstants ChannelInputTriggerSelect

// @constants ChannelInputTriggerMode

// channel input trigger modes
#define SA_CTL_CH_INPUT_TRIG_MODE_DISABLED                  0
#define SA_CTL_CH_INPUT_TRIG_MODE_EMERGENCY_STOP            1
#define SA_CTL_CH_INPUT_TRIG_MODE_STOP_TRIGGER              2

// @endconstants ChannelInputTriggerMode

// @constants ChannelOutputTriggerMode

// output trigger modes
#define SA_CTL_CH_OUTPUT_TRIG_MODE_CONSTANT                 0
#define SA_CTL_CH_OUTPUT_TRIG_MODE_POSITION_COMPARE         1
#define SA_CTL_CH_OUTPUT_TRIG_MODE_TARGET_REACHED           2
#define SA_CTL_CH_OUTPUT_TRIG_MODE_ACTIVELY_MOVING          3
#define SA_CTL_CH_OUTPUT_TRIG_MODE_IN_POSITION              4

// @endconstants ChannelOutputTriggerMode

// @constants TriggerCondition

// trigger conditions
#define SA_CTL_TRIGGER_CONDITION_RISING                     0
#define SA_CTL_TRIGGER_CONDITION_FALLING                    1
#define SA_CTL_TRIGGER_CONDITION_EITHER                     2

// @endconstants TriggerCondition

// @constants TriggerPolarity

// trigger polarities
#define SA_CTL_TRIGGER_POLARITY_ACTIVE_LOW                  0
#define SA_CTL_TRIGGER_POLARITY_ACTIVE_HIGH                 1

// @endconstants TriggerPolarity

/**********************************************************/
/* HM LOCK OPTIONS                                        */
/**********************************************************/

// @constants HM1LockOption

#define SA_CTL_HM1_LOCK_OPT_BIT_GLOBAL                      0x00000001
#define SA_CTL_HM1_LOCK_OPT_BIT_CONTROL                     0x00000002
#define SA_CTL_HM1_LOCK_OPT_BIT_CHANNEL_MENU                0x00000010
#define SA_CTL_HM1_LOCK_OPT_BIT_GROUP_MENU                  0x00000020
#define SA_CTL_HM1_LOCK_OPT_BIT_SETTINGS_MENU               0x00000040
#define SA_CTL_HM1_LOCK_OPT_BIT_LOAD_CFG_MENU               0x00000080
#define SA_CTL_HM1_LOCK_OPT_BIT_SAVE_CFG_MENU               0x00000100
#define SA_CTL_HM1_LOCK_OPT_BIT_CTRL_MODE_PARAM_MENU        0x00000200
#define SA_CTL_HM1_LOCK_OPT_BIT_CHANNEL_NAME                0x00001000
#define SA_CTL_HM1_LOCK_OPT_BIT_POS_TYPE                    0x00002000
#define SA_CTL_HM1_LOCK_OPT_BIT_SAFE_DIR                    0x00004000
#define SA_CTL_HM1_LOCK_OPT_BIT_CALIBRATE                   0x00008000
#define SA_CTL_HM1_LOCK_OPT_BIT_REFERENCE                   0x00010000
#define SA_CTL_HM1_LOCK_OPT_BIT_SET_POSITION                0x00020000
#define SA_CTL_HM1_LOCK_OPT_BIT_MAX_CLF                     0x00040000
#define SA_CTL_HM1_LOCK_OPT_BIT_POWER_MODE                  0x00080000
#define SA_CTL_HM1_LOCK_OPT_BIT_ACTUATOR_MODE               0x00100000
#define SA_CTL_HM1_LOCK_OPT_BIT_RANGE_LIMIT                 0x00200000
#define SA_CTL_HM1_LOCK_OPT_BIT_CONTROL_LOOP_INPUT          0x00400000
#define SA_CTL_HM1_LOCK_OPT_BIT_POS_CTRL_OPT                0x00800000
#define SA_CTL_HM1_LOCK_OPT_BIT_RESONANCE_FREQ              0x01000000

// @endconstants HM1LockOption

/**********************************************************/
/* EVENT NOTIFICATION OPTIONS                             */
/**********************************************************/

// @constants EventNotificationOption

#define SA_CTL_EVT_OPT_BIT_REQUEST_READY_ENABLED            0x00000001

// @endconstants EventNotificationOption

/**********************************************************/
/* POSITIONER TYPE                                        */
/**********************************************************/

// @constants PositionerType

#define SA_CTL_POSITIONER_TYPE_MODIFIED                     0
#define SA_CTL_POSITIONER_TYPE_AUTOMATIC                    299
#define SA_CTL_POSITIONER_TYPE_CUSTOM0                      250
#define SA_CTL_POSITIONER_TYPE_CUSTOM1                      251
#define SA_CTL_POSITIONER_TYPE_CUSTOM2                      252
#define SA_CTL_POSITIONER_TYPE_CUSTOM3                      253

// @endconstants PositionerType

/**********************************************************/
/* WRITE PROTECTION                                       */
/**********************************************************/

// @constants PosWriteProtection

#define SA_CTL_POS_WRITE_PROTECTION_KEY                     0x534D4152

// @endconstants PosWriteProtection

/**********************************************************/
/* MOVEMENT TYPES                                         */
/**********************************************************/

// @constants MovementType

#define SA_CTL_POS_MOVEMENT_TYPE_LINEAR                     0
#define SA_CTL_POS_MOVEMENT_TYPE_ROTATORY                   1
#define SA_CTL_POS_MOVEMENT_TYPE_GONIOMETER                 2
#define SA_CTL_POS_MOVEMENT_TYPE_TIP_TILT                   3
#define SA_CTL_POS_MOVEMENT_TYPE_IRIS                       4
#define SA_CTL_POS_MOVEMENT_TYPE_OSCILLATOR                 5
#define SA_CTL_POS_MOVEMENT_TYPE_HIGH_LOAD_TABLE            6

// @endconstants MovementType

/**********************************************************/
/* IO MODULE VOLTAGE                                      */
/**********************************************************/

// @constants IOModuleVoltage

#define SA_CTL_IO_MODULE_VOLTAGE_3V3                        0
#define SA_CTL_IO_MODULE_VOLTAGE_5V                         1

// @endconstants IOModuleVoltage

/**********************************************************/
/* IO MODULE OPTIONS                                      */
/**********************************************************/

// @constants IOModuleOption

#define SA_CTL_IO_MODULE_OPT_BIT_ENABLED                    0x00000001  // deprecated
#define SA_CTL_IO_MODULE_OPT_BIT_DIGITAL_OUTPUT_ENABLED     0x00000001
#define SA_CTL_IO_MODULE_OPT_BIT_EVENTS_ENABLED             0x00000002
#define SA_CTL_IO_MODULE_OPT_BIT_ANALOG_OUTPUT_ENABLED      0x00000004

// @endconstants IOModuleOption

/**********************************************************/
/* IO MODULE ANALOG INPUT RANGE                           */
/**********************************************************/

// @constants IOModuleAnalogInputRange

#define SA_CTL_IO_MODULE_ANALOG_INPUT_RANGE_BI_10V          0
#define SA_CTL_IO_MODULE_ANALOG_INPUT_RANGE_BI_5V           1
#define SA_CTL_IO_MODULE_ANALOG_INPUT_RANGE_BI_2_5V         2
#define SA_CTL_IO_MODULE_ANALOG_INPUT_RANGE_UNI_10V         3
#define SA_CTL_IO_MODULE_ANALOG_INPUT_RANGE_UNI_5V          4

// @endconstants IOModuleAnalogInputRange

/**********************************************************/
/* SENSOR MODULE OPTIONS                                  */
/**********************************************************/

// @constants SensorModuleOption

#define SA_CTL_SENSOR_MODULE_OPT_BIT_EVENTS_ENABLED         0x00000001

// @endconstants SensorModuleOption

/**********************************************************/
/* SIGNAL CORRECTION OPTIONS                              */
/**********************************************************/

// @constants SignalCorrectionOption

#define SA_CTL_SIGNAL_CORR_OPT_BIT_DAC                      0x00000002
#define SA_CTL_SIGNAL_CORR_OPT_BIT_DPEC                     0x00000008
#define SA_CTL_SIGNAL_CORR_OPT_BIT_ASC                      0x00000010

// @endconstants SignalCorrectionOption

/**********************************************************/
/* NETWORK DISCOVER MODE                                  */
/**********************************************************/

// @constants NetworkDiscoverMode

#define SA_CTL_NETWORK_DISCOVER_MODE_DISABLED               0
#define SA_CTL_NETWORK_DISCOVER_MODE_PASSIVE                1
#define SA_CTL_NETWORK_DISCOVER_MODE_ACTIVE                 2

// @endconstants NetworkDiscoverMode

/**********************************************************/
/* REFERENCE TYPES                                        */
/**********************************************************/

// @constants ReferenceType

#define SA_CTL_REF_TYPE_NONE                                0
#define SA_CTL_REF_TYPE_END_STOP                            1
#define SA_CTL_REF_TYPE_SINGLE_CODED                        2
#define SA_CTL_REF_TYPE_DISTANCE_CODED                      3

// @endconstants ReferenceType

/**********************************************************/
/* POSITIONER FAULT REASON BITS                           */
/**********************************************************/

// @constants PositionerFaultReason

#define SA_CTL_POS_FAULT_REASON_BIT_U_PHASE_SHORT           0x00000001
#define SA_CTL_POS_FAULT_REASON_BIT_V_PHASE_SHORT           0x00000002
#define SA_CTL_POS_FAULT_REASON_BIT_W_PHASE_SHORT           0x00000004
#define SA_CTL_POS_FAULT_REASON_BIT_U_PHASE_OPEN            0x00000008
#define SA_CTL_POS_FAULT_REASON_BIT_V_PHASE_OPEN            0x00000010
#define SA_CTL_POS_FAULT_REASON_BIT_W_PHASE_OPEN            0x00000020
#define SA_CTL_POS_FAULT_REASON_BIT_CURRENT_DEVIATION       0x00000040
#define SA_CTL_POS_FAULT_REASON_BIT_OUTPUT_SHORT            0x00000080
#define SA_CTL_POS_FAULT_REASON_BIT_DRIVER_FAULT            0x00008000

// @endconstants PositionerFaultReason

#endif // SMARACT_CTL_CONSTANTS_H
