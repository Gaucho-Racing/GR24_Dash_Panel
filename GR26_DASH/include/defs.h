#include <stdint.h>
/** [Byte 0 / Bits 0-7]
0: TS Active
1: RTD
2-7: Reserved (Byte 0) */
/** [Byte 1 / Bits 8-15]
0: BMS
1: IMD
2: BSPD
3-7: Reserved (Byte 1) */
typedef struct {
	union
	{
		struct
		{
		bool TS_Active : 1;
		bool RTD : 1;
		bool TS_Off : 1;
		bool RTD_Off : 1;
		bool reserved : 4;
		} BTN;
		uint8_t button_flags;
	} BF;
	union
	{
		struct
		{
		bool BMS : 1;
		bool IMD : 1;
		bool reserved : 6;
		} LED;
		uint8_t led_flags;
	} LB;
} GRCAN_DASH_STATUS_MSG;

/** Dash Config */
	/** [Byte 0 / Bits 0-7]
0: BMS LED command
1: IMD LED command
2: BSPD LED command
3-7: Reserved (Byte 0) */
typedef struct {
	union 
	{
		struct
		{
		bool BMS : 1;
		bool IMD : 1;
		bool BSPD : 1;
		bool reserved : 5;
		} LED;
		uint8_t led_flags;
	} LF;
} GRCAN_DASH_CONFIG_MSG;

//0x00200300
typedef struct {
	/** Time in millis (Byte 0) */
	uint32_t timestamp;
} GRCAN_PING_MSG;

//0x00200200
/** [Bytes 0-1 / Bits 0-15]
0: BCU Node Status (1: OK, 0: Timeout)
1: GR Inverter Status (1: OK, 0: Timeout)
2: Fan Controller 1 Status (1: OK, 0: Timeout)
3: Fan Controller 2 Status (1: OK, 0: Timeout)
4: Fan Controller 3 Status (1: OK, 0: Timeout)
5: Dash Panel Status (1: OK, 0: Timeout)
6: TCM Node Status (1: OK, 0: Timeout)
7: SAMM_Mag_1 Status (1: OK, 0: Timeout)
8: SAMM_Mag_2 Status (1: OK, 0: Timeout)
9: SAMM_ToF_1 Status (1: OK, 0: Timeout)
10: SAMM_ToF_2 Status (1: OK, 0: Timeout)
11: TireTemp_FL Status (1: OK, 0: Timeout)
12: TireTemp_FR Status (1: OK, 0: Timeout)
13: TireTemp_RL Status (1: OK, 0: Timeout)
14: TireTemp_RR Status (1: OK, 0: Timeout)
15: Reserved (Byte 0) 
[Byte 2 / Bits 16-17] GLV States
0: GLV Off State,
1: GLV On State.
See diagram in StateMachine.
[Byte 2 / Bits 18-19] Precharge States
2: Precharge Engaged State
3: Precharge Complete State
See diagram in StateMachine.h
[Byte 2 / Bits 20-21] ECU States
4: Drive Active ECU State
5: TS Discharge ECU State
6-7: Reserved
See diagram in StateMachine.h (Byte 2) */
typedef struct {
	uint16_t status_flags;
	uint8_t ecu_state;
	/** Controls the AC current limits to each of the inverters
Discrete Mapping, actual values TBD (16 possible values) The torque map selected; torque map is the mapping of the throttle to the torque sent to each motor (Byte 3) */
	uint8_t power_level_torque_map;
	/** the temperature of the hottest cell of the accumulator (Byte 4) */
	uint8_t max_cell_temp;
	/** % charged of the Accumulator (Byte 5) */
	uint8_t accumulator_state_of_charge;
	/** % charged of the Low Voltage Bat (Byte 6) */
	uint8_t glv_state_of_charge;
} GRCAN_ECU_STATUS_1_MSG;

enum GR_ECU_State {
	/**
	 * The GLV is off and not operational (not possible if ECU is running)
	 */
	GR_GLV_OFF = 0b00000001, //1
	/**
	 * The ECU is on and ready for operation.
	 */
	GR_GLV_ON = 0b00000010, //2
	/**
	 * The HV precharging process has been initiated.
	 */
	GR_PRECHARGE_ENGAGED = 0b00000100, //4
	/**
	 * The HV precharging process has been completed successfully.
	 */
	GR_PRECHARGE_COMPLETE = 0b00001000, //8
	/**
	 * The HV system is fully operational and the drive system is active.
	 */
	GR_DRIVE_ACTIVE = 0b00010000, //16
	/**
	 * The HV system is in the process of discharging, TS Voltage >60V.
	 */
	GR_TS_DISCHARGE = 0b00100000, //32
};

// 0x0500002
/*
ts_Active is first bit
RTD is second bit
*/
union Buttons
{
  struct
  {
    bool TS_Active : 1;
    bool RTD : 1;
    bool TS_Off : 1;
    bool RTD_Off : 1;
    bool reserved : 6;
  } BTN;
  uint8_t button_flags;
};