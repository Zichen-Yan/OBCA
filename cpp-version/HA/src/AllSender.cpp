// #include <iostream>
// #include <stdio.h>
// #include <stdlib.h>
// #include <dlfcn.h>
// #include <stdint.h>
// #include <signal.h>
// #include <sys/types.h>
// #include <sys/socket.h>
// #include <sys/un.h>
// #include <unistd.h>

// #include "APA_API.h"
// #include "ApplFunc.h"
// #include "rtwtypes.h"

// /*CAN���߹���ͷ�ļ�*/
// extern "C" {
// #include "./CANBus/SigMgrClient/SigMap_Hev.h"
// #include "./CANBus/SigMgrClient/SigMgrClient.h"
// }

// /*�ڴ湦��ͷ�ļ�*/
// #include "./DataBus/DatabusService.h"
// #include "./DataBus/Semaphore.h"

#include "log.h"

// using namespace std;
// using namespace bydDDS;


int main(void){  
  
  bydapa::common::Log::GetInstance()->SetFilename("Sender", false);
	LOG_DEBUG("Log.SetFilename:Sender");

  // int PathDataBusFlag = 0;
  // int CtrlDataBusFlag = 0;
  // int FusnDataBusFlag = 0;
  // int NavDataBusFlag = 0;
  // int HmiDataBusFlag = 0;
	
  // Domain1800 Apa_Path;
  // Domain1900 Apa_Ctrl;
  // Domain2000 Apa_Fusn;
  // Domain2500 Apa_Nav;
  // Domain2600 Apa_Hmi;
	
  // /*路径规划模块输入信号结构体*/
  // bydDDS::DataBus PathPlanning;
  // PathPlanning.domain = 1800;
  // PathPlanning.size = sizeof(Domain1800);
  // /*车辆控制模块输入信号结构体*/
  // bydDDS::DataBus VehicleControl;
  // VehicleControl.domain = 1900;
  // VehicleControl.size = sizeof(Domain1900);
  // /*融合模块输入信号结构体*/
  // bydDDS::DataBus Fusion;
  // Fusion.domain = 2000;
  // Fusion.size = sizeof(Domain2000);
  // /*Nav模块输入信号结构体*/
  // bydDDS::DataBus Navigation;
  // Navigation.domain = 2500;
  // Navigation.size = sizeof(Domain2500);
  // /*Hmi模块输入信号结构体*/
  // bydDDS::DataBus HmiDisplay;
  // HmiDisplay.domain = 2600;
  // HmiDisplay.size = sizeof(Domain2600);

  // PathDataBusFlag = PathPlanning.DadabusInit(PathPlanning.domain, PathPlanning.size);
  // if (PathDataBusFlag == -1) return 1;
  // //cout << "########PathDataBusFlag########:"<< PathDataBusFlag << endl;	

  // CtrlDataBusFlag = VehicleControl.DadabusInit(VehicleControl.domain, VehicleControl.size);
  // if (CtrlDataBusFlag == -1) return 1;
  // //cout << "########CtrlDataBusFlag########:"<< CtrlDataBusFlag << endl;		

  // FusnDataBusFlag = Fusion.DadabusInit(Fusion.domain, Fusion.size);
  // if (FusnDataBusFlag == -1) return 1;
  // //cout << "########FusnDataBusFlag########:"<< FusnDataBusFlag << endl;	

  // NavDataBusFlag = Navigation.DadabusInit(Navigation.domain, Navigation.size);
  // if (NavDataBusFlag == -1) return 1;	
  // // //cout << "########NavDataBusFlag########:"<< NavDataBusFlag << endl;	

  // HmiDataBusFlag = HmiDisplay.DadabusInit(HmiDisplay.domain, HmiDisplay.size);
  // if (HmiDataBusFlag == -1) return 1;	
  // //cout << "########HmiDataBusFlag########:"<< HmiDataBusFlag << endl;	
  
  // int b = 0;
  
  while(1){

    // Apa_Path.IsPlanningCompleted = 0;
    // Apa_Path.PlanningStatus = 1;


    // Apa_Ctrl.ObstacleOnPath = 0;
    
    b++;
    LOG_DEBUG("b=%d", b);
    if (b < 500){
    Apa_Hmi.ButtonType = 0;
    LOG_DEBUG("Apa_Hmi.ButtonType1=%d,%d",b, Apa_Hmi.ButtonType);
    // for(int i=0;i<25;i++){   
    // Apa_Fusn.parkingSpaceInfo[0].id = 1;
    // Apa_Fusn.parkingSpaceInfo[i].P0_X = i-100;
    // Apa_Fusn.parkingSpaceInfo[i].ParkingSpaceValid = 1;
   
    // }
    
    }

    // if (b > 500 && b < 1000){
    
    // // for(int i=0;i<25;i++){   
    // // Apa_Fusn.parkingSpaceInfo[1].id = 2;
    // // Apa_Fusn.parkingSpaceInfo[i].P0_X = i-100;
    // // Apa_Fusn.parkingSpaceInfo[i].ParkingSpaceValid = 1;
    
    // // }
    
    // }

    if (b >= 1000){
   
   // cout << "########HmiDataBusFlag########:"<< (int)Apa_Hmi.ButtonType<< ",b="<< b << endl;
    Apa_Hmi.SelectNum = 1; 
    
    
    LOG_DEBUG("Apa_Hmi.ButtonType2=%d,%d",b, Apa_Hmi.ButtonType);

    // for(int i=0;i<25;i++){   
    // Apa_Fusn.parkingSpaceInfo[2].id = 3;
    // Apa_Fusn.parkingSpaceInfo[i].P0_X = i-100;
    // Apa_Fusn.parkingSpaceInfo[i].ParkingSpaceValid = 1;

    // }
    
    }


    if (b >= 1200){
   
   // cout << "########HmiDataBusFlag########:"<< (int)Apa_Hmi.ButtonType<< ",b="<< b << endl;
    

    Apa_Hmi.ButtonType = 1;
    LOG_DEBUG("b=%d,Apa_Hmi.ButtonType3=%d",b, Apa_Hmi.ButtonType);

    // for(int i=0;i<25;i++){   
    // Apa_Fusn.parkingSpaceInfo[2].id = 3;
    // Apa_Fusn.parkingSpaceInfo[i].P0_X = i-100;
    // Apa_Fusn.parkingSpaceInfo[i].ParkingSpaceValid = 1;

    // }
    
    }





    Apa_Nav.nav_status = 1;
    Apa_Nav.nav_pos_X = 336;
    Apa_Nav.nav_pos_Y = 8;
    Apa_Nav.nav_heading = 32;


    // Apa_Hmi.ButtonType = 1;
	  // Apa_Hmi.SelectNum = 1;

    //cout << "########Apa_Hmi.ButtonType########:"<< (int)Apa_Hmi.ButtonType << endl;	
    // PathPlanning.DatabusSendData(PathPlanning.domain, PathPlanning.size, &Apa_Path, sizeof(Domain1800));
    // VehicleControl.DatabusSendData(VehicleControl.domain, VehicleControl.size, &Apa_Ctrl, sizeof(Domain1900));
    // Fusion.DatabusSendData(Fusion.domain, Fusion.size, &Apa_Fusn, sizeof(Domain2000));
    Navigation.DatabusSendData(Navigation.domain, Navigation.size, &Apa_Nav, sizeof(Domain2500));

    //cout << "########HmiDataBusFlag########:"<< (int)Apa_Hmi.ButtonType<< ",b="<< b << endl;	
    
    HmiDisplay.DatabusSendData(HmiDisplay.domain, HmiDisplay.size, &Apa_Hmi, sizeof(Domain2600));	  
    

	usleep(10000);
	}

  return 0;
}

