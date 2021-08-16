using System;
using System.Collections.Generic;
using System.Text;
using System.Threading.Tasks;

namespace TPCWare.LoRaWAN
{
    interface IUartLoRaWan
    {
        string LoRaWanVersion { get; }
        byte[] LoRaWanParameters { get; }
        bool SaveParamsOnPwrDown { get; }
        int ModuleAddress { get; }
        UartLoRaWanDataFrame DataFrame { get; }
        int BaudRate { get; }
        int AirDataRate { get; }
        int ChannelMHz { get; }
        UartLoRaWanTransmissionType TransmissionType { get; }
        UartLoRaWanIoDriveMode IoDriveMode { get; }
        int WakeUpTimeMs { get; }
        bool ForwardErrorCorrection { get; }
        int TransmissionPowerDb { get; }

        event EventHandler<byte[]> DataReceived;

        public Task ResetModule();
        public Task WriteToSerialPortAsync(string utf8Data);
        public Task WriteToSerialPortAsync(byte[] bytes);
        public Task RequestVersion();
        public Task RequestConfigParams();
        public Task SetParameters(bool saveParamsOnPwrDown = false,
                                  int moduleAddress = 0x0000,
                                  UartLoRaWanDataFrame dataFrame = UartLoRaWanDataFrame.EightNoneOne,
                                  int baudRate = 9600,
                                  int airDataRate = 2400,
                                  int channelMHz = 6,
                                  UartLoRaWanTransmissionType transmissionType = UartLoRaWanTransmissionType.Trasparent,
                                  UartLoRaWanIoDriveMode ioDriveMode = UartLoRaWanIoDriveMode.PushPullUp,
                                  int wakeUpTimeMs = 250,
                                  bool forwardErrorCorrection = true,
                                  int transmissionPowerDb = 20);
    }
}
