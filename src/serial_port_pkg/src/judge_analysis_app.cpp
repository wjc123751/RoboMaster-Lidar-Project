#define __DRIVER_GLOBALS

#include "judge_analysis_app.h"

void judgeCalculate(uint8_t JudgeDataBuffer[JudgeBufferLength])//����ϵͳ����
{
  static uint16_t start_pos=0,next_start_pos=0;
  while(1)
    {
      memcpy(&JudgementData.frameHeader, &JudgeDataBuffer[start_pos],FrameHeader_Len);
      /*��У��ͷ֡0xA5 Ȼ��crc8У��֡ͷ ��crc16λУ������*/
      if((JudgementData.frameHeader.SOF==(uint16_t)JudgeFrameHeader)\
          &&(1==Verify_CRC8_Check_Sum(&JudgeDataBuffer[start_pos],FrameHeader_Len))\
          &&(1==Verify_CRC16_Check_Sum(&JudgeDataBuffer[start_pos], JudgementData.frameHeader.DataLenth+FrameHeader_Len+4)))//����λ����+֡ͷ����+�����볤��+У���볤��
        {
          memcpy(&JudgementData.rxCmdId, (&JudgeDataBuffer[start_pos]+5), sizeof(JudgementData.rxCmdId));
          JudgeDataBuffer[start_pos]++;//ÿ������һ�ξ���֡ͷ��һ��ֹ�ٴδ�����֡����
          next_start_pos=start_pos+9+JudgementData.frameHeader.DataLenth;//9Ϊ 5λ֡ͷ 2λ���ݳ��� 2У��λ

          switch(JudgementData.rxCmdId)
            {
            case ID_game_state://��ȡ����״̬���� 0x0001
            {
              memcpy(&JudgementData.GameState,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case ID_game_result://��ȡ������� 0x0002
            {
              memcpy(&JudgementData.GameResult,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case ID_game_robot_HP://��ȡ������Ѫ����0x0003
            {
              memcpy(&JudgementData.GameRobotHP,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case ID_event_data://��ȡ�����¼� 0x0101
            {
              memcpy(&JudgementData.EventData,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case ID_supply_projectile_action://��ȡ����վ���� 0x0102
            {
              memcpy(&JudgementData.SupplyProjectileAction,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case ID_referee_warning://0x0104,//���о���
            {
              memcpy(&JudgementData.RefereeWarn,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case ID_dart_remaining_time://0x0105,//���ڷ����������
            {
              memcpy(&JudgementData.DartRemain,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;

            case Robot_Status_ID://0x0201,//������״̬  �ȼ�
            {
              memcpy(&JudgementData.robot_status_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case power_heat_data_ID://0x0202,//ǹ������ ���̹���
            {
              memcpy(&JudgementData.power_heat_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            case robot_pos_ID://0x0203,//������λ������
            {
              memcpy(&JudgementData.robot_pos_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case robot_buff_ID: //0x0204,//buff
            {
              memcpy(&JudgementData.buff_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case robot_hurt_ID://=0x0206,//�˺�����
            {
              memcpy(&JudgementData.hurt_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case shoot_data_ID://0x0207,//��Ƶ����
            {
              memcpy(&JudgementData.shoot_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case projectile_allowance_ID: //0x0208,//��������
            {
              memcpy(&JudgementData.projectile_allowance_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case rfid_status_ID: //=0x0209,//������ RFID ģ��״̬
            {
              memcpy(&JudgementData.rfid_status_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case dart_client_cmd_ID:// =0x020A,//����ѡ�ֶ�ָ������
            {
              memcpy(&JudgementData.dart_client_cmd_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case ground_robot_position_ID: //	=0x020B,//���������λ������
            {
              memcpy(&JudgementData.ground_robot_position_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case radar_mark_data_ID://=0x020C,//�״��ǽ�������
            {
              memcpy(&JudgementData.radar_mark_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case sentry_info_ID://  =0x020D,//�ڱ�����������Ϣͬ��
            {
              memcpy(&JudgementData.sentry_info_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
            case radar_info_t_ID://=0x020E,//�״�����������Ϣͬ��
            {
              memcpy(&JudgementData.radar_info_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;

            case student_interactive_header_ID:
            {
              memcpy(&JudgementData.userinfo,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
            }
            break;
//				case controller_header_ID: //�Զ��������
//        {
//					memcpy(&JudgementData.controller_data_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
//				}
//				break;
//				case key_inpict_ID: //ͼ�����ڴ�����
//        {
//					memcpy(&JudgementData.robot_key_t,(&JudgeDataBuffer[start_pos]+7),JudgementData.frameHeader.DataLenth);//�������е����ݸ��Ƶ���Ӧ�Ľṹ����ȥ
//				}
//				break;

            }
          start_pos=next_start_pos;
        }
      else
        {
          start_pos=0;
          break;
        }
      /**���ͷָ��Խ�����˳�ѭ��**/
      if(start_pos>JudgeBufferLength)
        {
          start_pos=0;
          break;
        }
    }
}

uint8_t JudgeUARTtemp;








