/*
 * http_ssi.c
 *
 *  Created on: 11-Oct-2021
 *      Author: controllerstech
 */


#include"http_ssi.h"
#include "string.h"
#include "stdio.h"
#include "audio_player.h"
#include "lwip/tcp.h"
#include "lwip/apps/httpd.h"
#include "main.h"
#include "fatfs.h"
#include "stm32f4xx_hal.h"
extern UART_HandleTypeDef huart1;
extern FATFS fs;
extern struct netif gnetif;
//音量
int num;
int indx = 0;
/* we will use character "x", "y","z" as tag for SSI */
char const* TAGCHAR[]={"x", "y", "z"};
char const** TAGS=TAGCHAR;

uint16_t ssi_handler (int iIndex, char *pcInsert, int iInsertLen)
{
	switch (iIndex) {
		case 0:
			indx+=1;
			sprintf(pcInsert, "%d", indx);
			return strlen(pcInsert);
			break;
		case 1:
			indx+=1;
			sprintf(pcInsert, "%d", indx);
			return strlen(pcInsert);
			break;
		case 2:
			indx+=1;
			sprintf(pcInsert, "%d", indx);
			return strlen(pcInsert);
			break;
		default :
			break;
	}

	return 0;
}




/************************ CGI HANDLER ***************************/
const char *CGIForm_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *CGIVolume_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);
const char *CGISetIP_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[]);


const tCGI FORM_CGI = {"/alm/alarm/alarm.php", CGIForm_Handler};
const tCGI VOLUME_CGI = {"/volume.cgi", CGIVolume_Handler};
const tCGI SETIP_CGI = {"/setip.cgi", CGISetIP_Handler};
//是否打断
char interrupt[10];

//播放文件名
char name[100];

//音量
char volume[30];

//是否循环播放
char circule[10];
tCGI CGI_TAB[3];


const char *CGIForm_Handler(int iIndex, int iNumParams, char *pcParam[], char *pcValue[])
{
	//MX_SDIO_SD_Init();
	  //MX_I2S2_Init();

	//Audio_Set_Volume(20);

	//  HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_RESET);

	if (iIndex == 0)
	{
		for (int i=0; i<iNumParams; i++)
		{
			if (strcmp(pcParam[i], "cmd") == 0)  // if the fname string is found
			{
				memset(name, '\0', 30);
				strcpy(name, pcValue[i]);


			}

			else if (strcmp(pcParam[i], "interrupt") == 0)  // if the fname string is found
					{
						strcpy(interrupt, pcValue[i]);
					}



			else if (strcmp(pcParam[i], "circule") == 0)  // if the fname string is found
					{
						strcpy(circule, pcValue[i]);
					}

		}




		if(interrupt[0]=='1'){
		//置为0 可以打断
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_SET);
		Audio_Player_Start(name);
		HAL_UART_Transmit(&huart1,"GGG",3,100);

		}else{
			  if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_9) == 0){

				  //置为0不可以打断
			HAL_GPIO_WritePin(GPIOG,GPIO_PIN_9,GPIO_PIN_SET);
									//改用uart播放模块
									  //Audio_Player_Start(name);
				HAL_UART_Transmit(&huart1,"zzzzz",3,100);



			 }

		}


	}
	//有效，此段代码可以重启开发板
	//HAL_NVIC_SystemReset();          	/* 重启 */





//有效，此段代码可以重启网卡ca
//	ip4_addr_t ipaddr;
//	ip4_addr_t netmask;
//	ip4_addr_t gw;
//	uint8_t IP_ADDRESS[4];
//	uint8_t NETMASK_ADDRESS[4];
//	uint8_t GATEWAY_ADDRESS[4];
//	  IP_ADDRESS[0] = 192;
//	  IP_ADDRESS[1] = 168;
//	  IP_ADDRESS[2] = 3;
//	  IP_ADDRESS[3] = 244;
//	  NETMASK_ADDRESS[0] = 255;
//	  NETMASK_ADDRESS[1] = 255;
//	  NETMASK_ADDRESS[2] = 255;
//	  NETMASK_ADDRESS[3] = 0;
//	  GATEWAY_ADDRESS[0] = 192;
//	  GATEWAY_ADDRESS[1] = 168;
//	  GATEWAY_ADDRESS[2] = 3;
//	  GATEWAY_ADDRESS[3] = 1;
//	  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
//	  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
//	  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);
//	  netif_set_addr(&gnetif, &ipaddr, &netmask, &gw);
//	  /* 网口可以重启,也可以不重启 */
//	  netif_set_down(&gnetif);
//	  netif_set_up(&gnetif);


	return "";
}



const char *CGIVolume_Handler(int iIndex, int iNumParams, char *pcP[], char *pcV[])
{

		for (int i=0; i<iNumParams; i++)
		{
			if (strcmp(pcP[i], "volume2") == 0)  // if the fname string is found
			{
				memset(volume, '\0', 30);
				strcpy(volume, pcV[i]);
				num=atoi(volume);
				Audio_Set_Volume(num);
			}

		}



	return "/play.html";
}




const char *CGISetIP_Handler(int iIndex, int iNumParams, char *pcP[], char *pcV[])
{
	unsigned char ip[12];
	int a,b,c,d;
    unsigned char *ce;
    unsigned char *cf ;
    unsigned char *cg;
    unsigned char *ch ;


	for (int i=0; i<iNumParams; i++)
		{
			if (strcmp(pcP[i], "ip") == 0)  // if the fname string is found
			{
				sscanf(pcV[i],"%d.%d.%d.%d",&a,&b,&c,&d);

				   ce = (unsigned char *)a;
				 cf = (unsigned char *)b;
			cg = (unsigned char *)c;
				  ch = (unsigned char *)d;
				ip[0]=ce;
				ip[1]=cf;
				ip[2]=cg;
				ip[3]=ch;

			}

			else if (strcmp(pcP[i], "mask") == 0)  // if the fname string is found
			{
				sscanf(pcV[i],"%d.%d.%d.%d",&a,&b,&c,&d);
				   ce = (unsigned char *)a;
							 cf = (unsigned char *)b;
						cg = (unsigned char *)c;
							  ch = (unsigned char *)d;
							ip[4]=ce;
							ip[5]=cf;
							ip[6]=cg;
							ip[7]=ch;

			}else if(strcmp(pcP[i],"gateway")==0){

				sscanf(pcV[i],"%d.%d.%d.%d",&a,&b,&c,&d);
				   ce = (unsigned char *)a;
							 cf = (unsigned char *)b;
						cg = (unsigned char *)c;
							  ch = (unsigned char *)d;
							ip[8]=ce;
							ip[9]=cf;
							ip[10]=cg;
							ip[11]=ch;

			}
		}


		  	ip4_addr_t ipaddr;
		  	ip4_addr_t netmask;
		  	ip4_addr_t gw;
		  	uint8_t IP_ADDRESS[4];
		  	uint8_t NETMASK_ADDRESS[4];
		  	uint8_t GATEWAY_ADDRESS[4];
		  	  IP_ADDRESS[0] = ip[0];
		  	  IP_ADDRESS[1] = ip[1];
		  	  IP_ADDRESS[2] = ip[2];
		  	  IP_ADDRESS[3] = ip[3];
		  	  NETMASK_ADDRESS[0] = ip[4];
		  	  NETMASK_ADDRESS[1] = ip[5];
		  	  NETMASK_ADDRESS[2] =ip[6];
		  	  NETMASK_ADDRESS[3] = ip[7];
		  	  GATEWAY_ADDRESS[0] = ip[8];
		  	  GATEWAY_ADDRESS[1] = ip[9];
		  	  GATEWAY_ADDRESS[2] = ip[10];
		  	  GATEWAY_ADDRESS[3] = ip[11];
		  	  IP4_ADDR(&ipaddr, IP_ADDRESS[0], IP_ADDRESS[1], IP_ADDRESS[2], IP_ADDRESS[3]);
		  	  IP4_ADDR(&netmask, NETMASK_ADDRESS[0], NETMASK_ADDRESS[1] , NETMASK_ADDRESS[2], NETMASK_ADDRESS[3]);
		  	  IP4_ADDR(&gw, GATEWAY_ADDRESS[0], GATEWAY_ADDRESS[1], GATEWAY_ADDRESS[2], GATEWAY_ADDRESS[3]);
		  	  netif_set_addr(&gnetif, &ipaddr, &netmask, &gw);
		  	  /* 网口可以重启,也可以不重启 */
		  	  netif_set_down(&gnetif);
		  	  netif_set_up(&gnetif);
	 	 // f_mount(&fs,"0:",1);


	 	 // write_data(ip);


	return "/play.html";
}






void http_server_init (void)
{
	httpd_init();
	CGI_TAB[0] = FORM_CGI;
	CGI_TAB[1] = VOLUME_CGI;
	CGI_TAB[2] = SETIP_CGI;
	http_set_cgi_handlers (CGI_TAB, 3);
}
