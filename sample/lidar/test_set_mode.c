//
// The MIT License (MIT)
//
// Copyright (c) 2019 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include <stdio.h>
#include <stdlib.h>
#ifdef WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include <string.h>
#include "livox_sdk.h"

typedef enum
{
  kDeviceStateDisconnect = 0,
  kDeviceStateConnect = 1,
  kDeviceStateSampling = 2,
} DeviceState;

typedef struct
{
  uint8_t handle;
  DeviceState device_state;
  DeviceInfo info;
} DeviceItem;

DeviceItem devices[kMaxLidarCount];
uint32_t data_recveive_count[kMaxLidarCount];

/** Connect all the broadcast device. */
int lidar_count = 0;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize];

/** Connect the broadcast device in list, please input the broadcast code and modify the BROADCAST_CODE_LIST_SIZE. */
/*#define BROADCAST_CODE_LIST_SIZE  3
int lidar_count = BROADCAST_CODE_LIST_SIZE;
char broadcast_code_list[kMaxLidarCount][kBroadcastCodeSize] = {
  "000000000000002",
  "000000000000003",
  "000000000000004"
};*/

void OnCommonCommandCallback(livox_status status, uint8_t handle, uint8_t response, void *client_data)
{
  printf("status: %d.\n", status);
  printf("handle: %d.\n", handle);
  printf("response: %d.\n", response);
}

/** Receiving error message from Livox Lidar. */
void OnLidarErrorStatusCallback(livox_status status, uint8_t handle, ErrorMessage *message)
{
  static uint32_t error_message_count = 0;
  if (message != NULL)
  {
    ++error_message_count;
    if (0 == (error_message_count % 100))
    {
      printf("handle: %u\n", handle);
      printf("temp_status : %u\n", message->lidar_error_code.temp_status);
      printf("volt_status : %u\n", message->lidar_error_code.volt_status);
      printf("motor_status : %u\n", message->lidar_error_code.motor_status);
      printf("dirty_warn : %u\n", message->lidar_error_code.dirty_warn);
      printf("firmware_err : %u\n", message->lidar_error_code.firmware_err);
      printf("pps_status : %u\n", message->lidar_error_code.device_status);
      printf("fan_status : %u\n", message->lidar_error_code.fan_status);
      printf("self_heating : %u\n", message->lidar_error_code.self_heating);
      printf("ptp_status : %u\n", message->lidar_error_code.ptp_status);
      printf("time_sync_status : %u\n", message->lidar_error_code.time_sync_status);
      printf("system_status : %u\n", message->lidar_error_code.system_status);
    }
  }
}

/** Receiving point cloud data from Livox LiDAR. */
void GetLidarData(uint8_t handle, LivoxEthPacket *data, uint32_t data_num, void *client_data)
{
  if (data)
  {
    data_recveive_count[handle]++;
    if (data_recveive_count[handle] % 100 == 0)
    {
      /** Parsing the timestamp and the point cloud data. */
      uint64_t cur_timestamp = *((uint64_t *)(data->timestamp));
      if (data->data_type == kCartesian)
      {
        LivoxRawPoint *p_point_data = (LivoxRawPoint *)data->data;
      }
      else if (data->data_type == kSpherical)
      {
        LivoxSpherPoint *p_point_data = (LivoxSpherPoint *)data->data;
      }
      else if (data->data_type == kExtendCartesian)
      {
        LivoxExtendRawPoint *p_point_data = (LivoxExtendRawPoint *)data->data;
      }
      else if (data->data_type == kExtendSpherical)
      {
        LivoxExtendSpherPoint *p_point_data = (LivoxExtendSpherPoint *)data->data;
      }
      else if (data->data_type == kDualExtendCartesian)
      {
        LivoxDualExtendRawPoint *p_point_data = (LivoxDualExtendRawPoint *)data->data;
      }
      else if (data->data_type == kDualExtendSpherical)
      {
        LivoxDualExtendSpherPoint *p_point_data = (LivoxDualExtendSpherPoint *)data->data;
      }
      else if (data->data_type == kImu)
      {
        LivoxImuPoint *p_point_data = (LivoxImuPoint *)data->data;
      }
      else if (data->data_type == kTripleExtendCartesian)
      {
        LivoxTripleExtendRawPoint *p_point_data = (LivoxTripleExtendRawPoint *)data->data;
      }
      else if (data->data_type == kTripleExtendSpherical)
      {
        LivoxTripleExtendSpherPoint *p_point_data = (LivoxTripleExtendSpherPoint *)data->data;
      }
      // printf("data_type %d packet num %d\n", data->data_type, data_recveive_count[handle]);
    }
  }
}

/** Callback function of starting sampling. */
void OnSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data)
{
  printf("OnSampleCallback statue %d handle %d response %d \n", status, handle, response);
  if (status == kStatusSuccess)
  {
    if (response != 0)
    {
      devices[handle].device_state = kDeviceStateConnect;
    }
  }
  else if (status == kStatusTimeout)
  {
    devices[handle].device_state = kDeviceStateConnect;
  }
}

/** Callback function of stopping sampling. */
void OnStopSampleCallback(livox_status status, uint8_t handle, uint8_t response, void *data)
{
}

/** Query the firmware version of Livox LiDAR. */
void OnDeviceInformation(livox_status status, uint8_t handle, DeviceInformationResponse *ack, void *data)
{
  if (status != kStatusSuccess)
  {
    printf("Device Query Informations Failed %d\n", status);
  }
  if (ack)
  {
    printf("firm ver: %d.%d.%d.%d\n",
           ack->firmware_version[0],
           ack->firmware_version[1],
           ack->firmware_version[2],
           ack->firmware_version[3]);
  }
}

void LidarConnect(const DeviceInfo *info)
{
  uint8_t handle = info->handle;
  QueryDeviceInformation(handle, OnDeviceInformation, NULL);
  if (devices[handle].device_state == kDeviceStateDisconnect)
  {
    devices[handle].device_state = kDeviceStateConnect;
    devices[handle].info = *info;
  }
}

void LidarDisConnect(const DeviceInfo *info)
{
  uint8_t handle = info->handle;
  devices[handle].device_state = kDeviceStateDisconnect;
}

void LidarStateChange(const DeviceInfo *info)
{
  uint8_t handle = info->handle;
  devices[handle].info = *info;
}

/** Callback function of changing of device state. */
void OnDeviceInfoChange(const DeviceInfo *info, DeviceEvent type)
{
  if (info == NULL)
  {
    return;
  }

  uint8_t handle = info->handle;
  if (handle >= kMaxLidarCount)
  {
    return;
  }
  if (type == kEventConnect)
  {
    LidarConnect(info);
    printf("[WARNING] Lidar sn: [%s] Connect!!!\n", info->broadcast_code);
  }
  else if (type == kEventDisconnect)
  {
    LidarDisConnect(info);
    printf("[WARNING] Lidar sn: [%s] Disconnect!!!\n", info->broadcast_code);
  }
  else if (type == kEventStateChange)
  {
    LidarStateChange(info);
    printf("[WARNING] Lidar sn: [%s] StateChange!!!\n", info->broadcast_code);
  }

  if (devices[handle].device_state == kDeviceStateConnect)
  {
    printf("Device Working State %d\n", devices[handle].info.state);
    if (devices[handle].info.state == kLidarStateInit)
    {
      printf("Device State Change Progress %u\n", devices[handle].info.status.progress);
    }
    else
    {
      printf("Device State Error Code 0X%08x\n", devices[handle].info.status.status_code.error_code);
    }
    printf("Device feature %d\n", devices[handle].info.feature);
    SetErrorMessageCallback(handle, OnLidarErrorStatusCallback);
    if (devices[handle].info.state == kLidarStateNormal)
    {
      LidarStartSampling(handle, OnSampleCallback, NULL);
      devices[handle].device_state = kDeviceStateSampling;
    }
  }
}

/** Callback function when broadcast message received.
 * You need to add listening device broadcast code and set the point cloud data callback in this function.
 */
void OnDeviceBroadcast(const BroadcastDeviceInfo *info)
{
  if (info == NULL || info->dev_type == kDeviceTypeHub)
  {
    return;
  }

  printf("Receive Broadcast Code %s\n", info->broadcast_code);

  if (lidar_count > 0)
  {
    bool found = false;
    int i = 0;
    for (i = 0; i < lidar_count; ++i)
    {
      if (strncmp(info->broadcast_code, broadcast_code_list[i], kBroadcastCodeSize) == 0)
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      return;
    }
  }

  bool result = false;
  uint8_t handle = 0;
  result = AddLidarToConnect(info->broadcast_code, &handle);
  if (result == kStatusSuccess)
  {
    /** Set the point cloud data for a specific Livox LiDAR. */
    SetDataCallback(handle, GetLidarData, NULL);
    devices[handle].handle = handle;
    devices[handle].device_state = kDeviceStateDisconnect;
  }
}

bool ChangeMode()
{
  printf("Livox SDK initializing.\n");
  /** Initialize Livox-SDK. */
  // if (argc != 2)
  // {
  //   printf("Please use setmode [target_mode] .\n");
  // }
  // int mode_index = atoi(argv[1]);
  // printf("target mode: %d .\n", mode_index);
  // LidarMode mode = mode_index;

  if (!Init())
  {
    return -1;
  }
  printf("Livox SDK has been initialized.\n");

  printf("Livox SDK has been initialized.\n");

  LivoxSdkVersion _sdkversion;
  GetLivoxSdkVersion(&_sdkversion);
  printf("Livox SDK version %d.%d.%d .\n", _sdkversion.major, _sdkversion.minor, _sdkversion.patch);

  memset(devices, 0, sizeof(devices));
  memset(data_recveive_count, 0, sizeof(data_recveive_count));

  /** Set the callback function receiving broadcast message from Livox LiDAR. */
  SetBroadcastCallback(OnDeviceBroadcast);

  /** Set the callback function called when device state change,
   * which means connection/disconnection and changing of LiDAR state.
   */
  SetDeviceStateUpdateCallback(OnDeviceInfoChange);

  //

  /** Start the device discovering routine. */
  int start_count = 0;

  while (!Start() && start_count < 3)
  {
    start_count++;
    printf("Start error, try again, %d of 3.\n", start_count);
    sleep(5);
  }
  printf("Start success \n");

  if (start_count == 3)
  {
    Uninit();
    printf("No lidar connected found.\n");
    return -1;
  }

  printf("Start discovering device.\n");

  sleep(5);
  int i = 0;
  for (i = 0; i < kMaxLidarCount; ++i) // kMaxLidarCount = 32
  {
    //      if (devices[i].device_state == kDeviceStateSampling)
    //      {
    //          printf("******** Lidar device %d.\n", i);
    //          printf("*** handle: %d \n", devices[i].handle);
    //          printf("*** broadcast_code: [%s] \n", devices[i].info.broadcast_code);
    //          printf("*** ip: [%s] \n", devices[i].info.ip);
    //      }
    //      if(i < 3)
    if (devices[i].device_state > kDeviceStateDisconnect)
    {
      printf("******** Lidar device %d.\n", i);
      printf("*** handle: %d \n", devices[i].handle);
      printf("*** broadcast_code: [%s] \n", devices[i].info.broadcast_code);
      printf("*** ip: [%s] \n", devices[i].info.ip);
      int command;
      printf("Please input target mode: \n N or n : NormalWorking\n P or p : PowerSaving\n S or s : Standby\n Q or q : Quit\n");
      while ((command = getchar()) != EOF)
      {
        getchar();
        LidarMode mode_target = kLidarModeNormal;
        if (command == 'N' || command == 'n')
        {
          mode_target = kLidarModeNormal;
          printf("Target mode: Normal \n");
        }
        else if (command == 'P' || command == 'p')
        {
          mode_target = kLidarModePowerSaving;
          printf("Target mode: PowerSaving \n");
        }
        else if (command == 'S' || command == 's')
        {
          printf("Target mode: Standby \n");
          mode_target == kLidarModeStandby;
        }
        else if (command == 'Q' || command == 'q')
        {
          break;
        }
        else
        {
          printf("Error command, please try again \n");
          continue;
        }

        livox_status status;
        status = LidarSetMode(devices[i].handle, mode_target, OnCommonCommandCallback, NULL);
        LidarMode mode_cur = LidarGetMode(devices[i].handle);
        printf("Current lidar mode: %d.\n", mode_cur);
      }
      //  status = LidarSetMode(devices[i].handle, kLidarModeNormal, OnCommonCommandCallback, NULL);
      //          status = LidarTurnOnFan(devices[i].handle, OnCommonCommandCallback, NULL);
      //          status = LidarTurnOffFan(devices[i].handle, OnCommonCommandCallback, NULL);
      //          printf("*** status: %d \n", status);
      //          status = LidarTurnOffFan(devices[i].handle, OnCommonCommandCallback, NULL);
      // break;
      /** Uninitialize Livox-SDK. */
      Uninit();
      return true;
    }
    //  livox_status status;
    //  status = LidarSetMode(devices[handle].handle, kLidarModePowerSaving, OnCommonCommandCallback, NULL);
  }

  if (i == kMaxLidarCount)
  {
    printf("No lidar connected found.\n");
    /** Uninitialize Livox-SDK. */
    Uninit();
    return false;
  }
}

int main(int argc, const char *argv[])
{
  int try_count = 0;
  while (!ChangeMode() && try_count < 5)
  {
    sleep(2);
  }

  return 0;
}