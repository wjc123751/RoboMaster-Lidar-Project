#ifndef JUDGE_ANALYSIS_H
#define JUDGE_ANALYSIS_H

#include "string.h"
#include "crc_check.h"

/************************ 2024��3��Climber�����ϵͳ����  ***********************/
/*********************************************************************************/
/************************ 2017��Ĳ���ϵͳЭ��ṹ�� Start ***********************/

#ifdef  __DRIVER_GLOBALS
#define __DRIVER_EXT
#else
#define __DRIVER_EXT extern
#endif

#define JudgeBufferLength_PJ        29 //0X0004
#define JudgeBufferLength_Position  25 //0X0008
#define JudgeFrameLength_PJPosi_SH      54 //֮������54����Ϊ�󽮽�04 �� 08�������ݰ��ŵ�һ֡�﷢�ͣ�����У��ȴ������һ������У��
#define JudgeFrameLength_1_SH      54

#define JudgeFrameLength_2_SH      11
#define JudgeFrameLength_3_SH      24

/************************ 2017��Ĳ���ϵͳЭ��ṹ�� End ***********************/



/************************ 2024��3��Climber�����ϵͳ����  ***********************/
/*********************    �������н�����Ϣ   ************************************/
///******************  2024��Ĳ���ϵͳЭ��ṹ�� Start  ***********************/
#define JudgeBufferLength       400
#define JudgeFrameHeader        0xA5        //֡ͷ 
#define FrameHeader_Len         5

#define CMD_LEN      2    //cmdid bytes
#define CRC_LEN      2    //crc16 bytes
#define HEADER_LEN   sizeof(tFrameHeader)  //frame length

struct tFrameHeader
{
  uint8_t   SOF;//0xA5
  uint16_t  DataLenth;
  uint8_t   Seq;
  uint8_t   CRC8;
} __attribute__((packed));

typedef enum
{
  GameInfo = 0x0001,
  RealBloodChangedData,
  RealShootData,
  SelfDefinedData =0x0100,
  Wrong = 0x1301
} tCmdID;


/***************������ID********************/

/*

	ID: 0x0001  Byte:  3    ����״̬����       			����Ƶ�� 1Hz
	ID: 0x0002  Byte:  1    �����������         		������������
	ID: 0x0003  Byte:  2    ���������˴������   		1Hz����
	ID: 0x0101  Byte:  4    �����¼�����   				�¼��ı����
	ID: 0x0102  Byte:  3    ���ز���վ������ʶ����    	�����ı����
	ID: 0X0103  Byte:  2    ���ز���վԤԼ�ӵ�����      �����ӷ��ͣ�10Hz
	ID: 0X0201  Byte: 15    ������״̬����        		10Hz
	ID: 0X0202  Byte: 14    ʵʱ������������   			50Hz
	ID: 0x0203  Byte: 16    ������λ������           	10Hz
	ID: 0x0204  Byte:  1    ��������������           	����״̬�ı����
	ID: 0x0205  Byte:  3    ���л���������״̬����      10Hz
	ID: 0x0206  Byte:  1    �˺�״̬����           		�˺���������
	ID: 0x0207  Byte:  6    ʵʱ�������           		�ӵ��������
	ID: 0x0301  Byte:  n    �����˼佻������           	���ͷ���������,10Hz

*/

typedef enum
{
  ID_game_state       					= 0x0001,//����״̬����
  ID_game_result 	   						= 0x0002,//�����������
  ID_game_robot_HP       				= 0x0003,//���������˴������

  ID_event_data  								= 0x0101,//�����¼�����
  ID_supply_projectile_action   = 0x0102,//���ز���վ������ʶ����
  ID_referee_warning						=	0x0104,//���о���
  ID_dart_remaining_time				= 0x0105,//���ڷ����������

  Robot_Status_ID               =0x0201,//������״̬  �ȼ�
  power_heat_data_ID   	        =0x0202,//ǹ������ ���̹���
  robot_pos_ID   	 					    =0x0203,//������λ������
  robot_buff_ID   	 					  =0x0204,//buff
  robot_hurt_ID                 =0x0206,//�˺�����
  shoot_data_ID	      					=0x0207,//��Ƶ����
  projectile_allowance_ID	   		=0x0208,//��������
  rfid_status_ID   	 					  =0x0209,//������ RFID ģ��״̬
  dart_client_cmd_ID            =0x020A,//����ѡ�ֶ�ָ������
  ground_robot_position_ID	   	=0x020B,//���������λ������
  radar_mark_data_ID            =0x020C,//�״��ǽ�������
  sentry_info_ID   	      	 	  =0x020D,//�ڱ�����������Ϣͬ��
  radar_info_t_ID   	 					=0x020E,//�״�����������Ϣͬ��

  student_interactive_header_ID     =0x0301,
  controller_header_ID     =0x0302,
  key_inpict_ID     =0x0304,

} Judege_Cmd_ID;
//֡ͷ




//����״̬���� ������ID 0x0001  ����Ƶ�ʣ�1Hz
typedef struct
{
  uint8_t game_type:4;     //0-3bit:�������� 1����ʦ�� 2:������ 3���˹�������ս�� 4��3V3 5:1V1
  uint8_t game_progress:4; //4-7bit:�����׶� 0��δ��ʼ���� 1��׼���׶� 2���Լ�׶� 3:5s����ʱ 4����ս�� 5������������
  uint16_t stage_remain_time; //��ǰ�׶�ʣ��ʱ�� /s

  uint64_t SyncTimeStamp;
} ext_game_status_t;

//����������� ������ID 0x0002  ����Ƶ�ʣ�������������
typedef struct
{
  uint8_t winner;  //0:ƽ�� 1:�췽ʤ�� 2������ʤ��
} ext_game_result_t;

//������Ѫ������ ������ID 0x0003  ����Ƶ�ʣ�1Hz
typedef struct
{
  uint16_t red_1_robot_HP; //��1Ӣ�ۻ�����Ѫ��  δ�ϳ��Լ�����Ѫ��Ϊ0
  uint16_t red_2_robot_HP; //��2���̻�����Ѫ��
  uint16_t red_3_robot_HP; //��3����������Ѫ��
  uint16_t red_4_robot_HP; //��4����������Ѫ��
  uint16_t red_5_robot_HP; //��5����������Ѫ��
  uint16_t red_7_robot_HP; //��5�ڱ�������Ѫ��

  uint16_t red_outpost_HP; //�췽ǰ��վ
  uint16_t red_base_HP; //�췽����

  uint16_t blue_1_robot_HP; //��1Ӣ�ۻ�����Ѫ��  δ�ϳ��Լ�����Ѫ��Ϊ0
  uint16_t blue_2_robot_HP; //��2���̻�����Ѫ��
  uint16_t blue_3_robot_HP; //��3����������Ѫ��
  uint16_t blue_4_robot_HP; //��4����������Ѫ��
  uint16_t blue_5_robot_HP; //��5����������Ѫ��
  uint16_t blue_7_robot_HP; //��5�ڱ�������Ѫ��

  uint16_t blue_outpost_HP; //����ǰ��վ
  uint16_t blue_base_HP; //��������
} ext_game_robot_HP_t;

//������Ϣ      ������ID 0x0101
typedef struct
{
  uint32_t event_type;
} ext_event_data_t;

//����վ������ʶ ������ID 0x0102  ����Ƶ�ʣ������ı���� ���ͷ�Χ������������
typedef struct
{
  uint8_t reserved;
  uint8_t supply_robot_id;
  uint8_t supply_projectile_step;
  uint8_t supply_projectile_num;
} ext_supply_projectile_action_t;

//���о�����Ϣ ������ID 0x0104  ����Ƶ�ʣ������������
typedef struct
{
  uint8_t level;
  uint8_t offending_robot_id;
  uint8_t count;
} ext_referee_warning_t;


//���ڷ���ڵ���ʱ ������ID 0x0105  ����Ƶ�ʣ�1Hz  ���ͷ�Χ������������
typedef struct
{
  uint8_t dart_remaining_time;
  uint16_t dart_info;
} ext_dart_remaining_time_t;


/*����������״̬   ������ID 0x0201 */
typedef struct
{
  uint8_t robot_id;
  uint8_t robot_level;
  uint16_t current_HP;
  uint16_t maximum_HP;
  uint16_t shooter_barrel_cooling_value;
  uint16_t shooter_barrel_heat_limit;
  uint16_t chassis_power_limit;
  uint8_t power_management_gimbal_output : 1;
  uint8_t power_management_chassis_output : 1;
  uint8_t power_management_shooter_output : 1;

} ext_game_robot_state_t;

/*���̹��ʼ�ǹ������ ������ID 0x0202 */
typedef struct
{
  uint16_t chassis_volt;//���������ѹ  ��λ ����
  uint16_t chassis_current;//�����������  ��λ����
  float    chassis_power;//�����������  ��λ W
  uint16_t buffer_energy;//���̹��ʻ���  ��λJ
  uint16_t shooter_17_heat1;//17mmǹ������
  uint16_t shooter_17_heat2;//17mmǹ������
  uint16_t shooter_42_heat3;//42mmǹ������
} ext_power_heat_data_t;

/*���̹��ʼ�ǹ������ ������ID 0x0203 */
typedef struct
{
  float x;
  float y;
  float angle;
} ext_robot_pos_t;

//���������� ������ID 0x0204  ����Ƶ�ʣ�1Hz
typedef struct
{
  uint8_t recovery_buff;
  uint8_t cooling_buff;
  uint8_t defence_buff;
  uint8_t vulnerability_buff;
  uint16_t attack_buff;
} ext_buff_t;

//����֧Ԯʱ������ ������ID 0x0205  ����Ƶ�ʣ�1Hz
typedef struct
{
  uint8_t airforce_status;
  uint8_t time_remain;
} ext_air_support_data_t;

/*�˺�����   ������ID 0x0206 */
typedef struct
{
  uint8_t armor_id : 4;
  uint8_t HP_deduction_reason : 4;
} ext_robot_hurt_t;

/*ʵʱ�������  ������ID 0x0207*/
typedef struct
{
  uint8_t bullet_type;
  uint8_t shooter_number;
  uint8_t launching_frequency;
  float initial_speed;
} ext_shoot_data_t;

/*��������  ������ID 0x0208*/
typedef struct
{
  uint16_t projectile_allowance_17mm;
  uint16_t projectile_allowance_42mm;
  uint16_t remaining_gold_coin;
} ext_projectile_allowance_t;

/*������ RFID ģ��״̬ ������ID 0x0209*/
typedef struct
{
  uint32_t rfid_status;
} ext_rfid_status_t;

/*����ѡ�ֶ�ָ������  ������ID 0x020A*/
typedef struct
{
  uint8_t dart_launch_opening_status;
  uint8_t reserved;
  uint16_t target_change_time;
  uint16_t latest_launch_cmd_time;
} ext_dart_client_cmd_t;

/*���������λ������  ������ID 0x020B*/
typedef struct
{
  float hero_x;
  float hero_y;
  float engineer_x;
  float engineer_y;
  float standard_3_x;
  float standard_3_y;
  float standard_4_x;
  float standard_4_y;
  float standard_5_x;
  float standard_5_y;
} ext_ground_robot_position_t;

/*�״��ǽ�������  ������ID 0x020C*/
struct ext_radar_mark_data_t
{
  uint8_t mark_hero_progress;
  uint8_t mark_engineer_progress;
  uint8_t mark_standard_3_progress;
  uint8_t mark_standard_4_progress;
  uint8_t mark_standard_5_progress;
  uint8_t mark_sentry_progress;
} __attribute__((packed));

/*�ڱ�����������Ϣͬ��  ������ID 0x020D*/
typedef struct
{
  uint32_t sentry_info;
} ext_sentry_info_t;

/*�״�����������Ϣͬ��  ������ID 0x020E*/
struct ext_radar_info_t
{
  uint8_t radar_info;
} __attribute__((packed));




/*�û��Զ��岿�ֿ��Ի�UI��ʾ����Ļ��  ������ID 0x0301*/
/*0-1 ��������ID 0xD180
	2-3 �����ߵ�ID
*/

typedef struct
{
  uint16_t rxCmdId;
  uint16_t data_id;
  uint16_t send_id;
  uint16_t receive_id;
} id_data_t;

typedef struct
{
  tFrameHeader Header;
  id_data_t    id;
  float data1;
  float data2;
  float data3;
  uint8_t masks;
  uint16_t crc_16;
} ext_client_custom_data_t;

//������֮���໥ͨ��  ������ID:0x0200~0x02FF       ע�⣬����UI�ṹ�嶨����judge_send_app.h����
typedef struct
{
  uint8_t data[112];
} robot_interactive_data_t;

//�ڱ���������ָ��  ������ID:0x0120
typedef struct
{
  uint32_t sentry_cmd;
} ext_sentry_cmd_t;

//�״���������ָ��  ������ID:0x0121
struct ext_radar_cmd_t
{
  uint32_t sentry_cmd;
} __attribute__((packed));

//�Զ��������x0302
typedef struct
{
  uint8_t data[30];
} ext_robot_controller_data_t;


/*ѡ�ֶ�С��ͼ��������  ������ID 0x0303*/
typedef struct
{
  float target_position_x;
  float target_position_y;
  uint8_t cmd_keyboard;
  uint8_t target_robot_id;
  uint8_t cmd_source;
} ext_map_command_t;

/*ͼ��������ϸ��ͼ�����󴮿��ļ�  ������ID 0x0304*/

/*ѡ�ֶ�С��ͼ�����״�����  ������ID 0x0305*/
struct ext_map_robot_data_t
{
  uint16_t target_robot_id;
  float target_position_x;
  float target_position_y;
} __attribute__((packed));

struct points_map_t
{
  tFrameHeader Header;
  uint16_t cmd_id;
  ext_map_robot_data_t mapdata;
  uint16_t crc;
} __attribute__((packed));


/*ѡ�ֶ�С��ͼ�����ڱ�����  ������ID 0x0307*/
typedef struct
{
  uint8_t intention;
  uint16_t start_position_x;
  uint16_t start_position_y;
  int8_t delta_x[49];
  int8_t delta_y[49];
  uint16_t sender_id;
} ext_map_data_t;

/*ѡ�ֶ�С��ͼ���ջ���������  ������ID 0x0308*/
typedef struct
{
  uint16_t sender_id;
  uint16_t receiver_id;
  uint8_t user_data[30];
} ext_custom_info_t;

typedef struct
{
  int16_t mouse_x;
  int16_t mouse_y;
  int16_t mouse_z;
  int8_t left_button_down;
  int8_t right_button_down;
  uint16_t keyboard_value;
  uint16_t reserved;
} ext_robot_command_t;


typedef  struct
{
  tFrameHeader          frameHeader;
  uint16_t              rxCmdId;

  ext_game_status_t     GameState;				        	//0x0001
  ext_game_result_t     GameResult;			        	  //0x0002
  ext_game_robot_HP_t   GameRobotHP;			          //0x0003

  ext_event_data_t        			   EventData;					        //0x0101
  ext_supply_projectile_action_t	 SupplyProjectileAction;		//0x0102
  ext_referee_warning_t            RefereeWarn;               //0x0104
  ext_dart_remaining_time_t        DartRemain;                //0x0105


  ext_game_robot_state_t  robot_status_t;			 //0x0201
  ext_power_heat_data_t   power_heat_data_t;	 //0x0202
  ext_robot_pos_t					robot_pos_t;				 //0x0203
  ext_buff_t              buff_t;			   			 //0x0204
  ext_robot_hurt_t        hurt_data_t; 			   //0x0206
  ext_shoot_data_t		    shoot_data_t;		     //0x0207
  ext_projectile_allowance_t projectile_allowance_t; //0x0208
  ext_rfid_status_t				rfid_status_t;			 //0x0209
  ext_dart_client_cmd_t   dart_client_cmd_t;   //0x020A
  ext_ground_robot_position_t ground_robot_position_t; //0x020B
  ext_radar_mark_data_t   radar_mark_data_t;   //0x020C
  ext_sentry_info_t 			sentry_info_t;       //0x020D
  ext_radar_info_t 				radar_info_t;   	   //0x020E


  ext_robot_command_t     robot_key_t;
  ext_robot_controller_data_t controller_data_t;
  ext_client_custom_data_t robot_data_t;
  ext_client_custom_data_t userinfo;
} JudgementDataTypedef;

///******************  2024��Ĳ���ϵͳЭ��ṹ�� end  ***********************/




typedef struct
{
  float data1;
  float data2;
  float data3;
  uint8_t mask;
} tSelfDefineInfo;

typedef  struct
{
  float data1;
  float data2;
  float data3;
  uint8_t mask;
} SelfDefineInfo_t;  //ѧ���ϴ��Զ������� (0x0005)

typedef  struct
{
  tFrameHeader          frameHeader;
  uint16_t              rxCmdId;
  union
  {
    ext_power_heat_data_t power_heat_data_t;
    ext_shoot_data_t		  shoot_data_t;
    ext_robot_hurt_t      robot_hurt_t;
    ext_game_robot_state_t game_robot_state_t;
    robot_interactive_data_t robot_data_t;
    ext_client_custom_data_t userinfo;
  } Data;
  uint16_t        CRC16;
  uint16_t        CRC16_2 ;
} FRAME;

typedef struct
{
  tFrameHeader    FrameHeader;
  tCmdID          CmdID;
  tSelfDefineInfo SelfDefineInfo;
  uint16_t        CRC16;
} tFrame;



__DRIVER_EXT uint8_t JudgeDataBuffer[JudgeBufferLength];
__DRIVER_EXT SelfDefineInfo_t SelfDefineInfo;
__DRIVER_EXT JudgementDataTypedef JudgementData;

void Send_FrameData(uint16_t cmdid, uint8_t * pchMessage,uint16_t dwLength);
void client_send(uint8_t * data);
void client_init(uint8_t * data1,uint8_t * data2);
void get_chassis_power_and_buffer(float *power, float *buffer, uint16_t *powmax);
void judgeCalculate(uint8_t JudgeDataBuffer[JudgeBufferLength]);//����ϵͳ����



#endif
