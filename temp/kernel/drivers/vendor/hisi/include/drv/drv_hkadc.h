/******************************************************************************

                  ��Ȩ���� (C), 2001-2011, ��Ϊ�������޹�˾

 ******************************************************************************
  �� �� ��   : drv_hkadc.h
  �� �� ��   : ����
  ��    ��   : �ž�Զ
  ��������   : 2013��2��2��
  ����޸�   :
  ��������   : drv_hkadc.h ��ͷ�ļ�
  �����б�   :

******************************************************************************/

#ifndef __DRV_HKADC_H__
#define __DRV_HKADC_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "drv_comm.h"


/*****************************************************************************
* �� �� ��  : BSP_HKADC_PaValueGet
*
* ��������  :
*
* �������  :
* �������  : ��
*
* �� �� ֵ  : ��
*
* �޸ļ�¼  : 2011-3-29 wuzechun creat
*
*****************************************************************************/
BSP_S32 BSP_HKADC_PaValueGet( BSP_U16 *pusValue );


/*************************************************
 �� �� ��   : DRV_HKADC_BAT_VOLT_GET
 ��������   : ���ص�ǰ��ص�ѹֵ
 �������   : pslData : ��ص�ѹֵ
 �������   : pslData : ��ص�ѹֵ
 �� �� ֵ   :0:��ȡ�ɹ�
            -1:��ȡʧ��
*************************************************/
BSP_S32 DRV_HKADC_BAT_VOLT_GET(BSP_S32 *ps32Data);


/*****************************************************************************
 �� �� ��  : hkadcBatADCRead
 ��������  : ��ȡ��ص�ѹ����ֵ
 �������  : ��
 �������  : pTemp��        ָ�����¶ȵ�ָ�롣
 �� �� ֵ  : 0:  �����ɹ���
             -1������ʧ�ܡ�
*****************************************************************************/
BSP_S32 DRV_GET_BATTERY_ADC(BSP_S32 * pslData);

#ifdef __cplusplus
}
#endif
#endif
