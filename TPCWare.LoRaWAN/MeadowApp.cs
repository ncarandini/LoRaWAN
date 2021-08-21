using Meadow;
using Meadow.Devices;
using Meadow.Foundation;
using Meadow.Foundation.Leds;
using Meadow.Hardware;
using System;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Threading.Tasks;

namespace TPCWare.LoRaWAN
{
    public class MeadowApp : App<F7Micro, MeadowApp>
    {
        private IUartLoRaWan uartLoRaWanDevice;

        private StringBuilder msg;

        public MeadowApp()
        {
            Initialize();
            var task = ManageLoRaWanCommunications();
            task.GetAwaiter().GetResult();

            // Loop
            while (true)
            {
                Thread.Sleep(1000);
            }
        }

        void Initialize()
        {
            Console.WriteLine("Initialize LoRaWAN E32 868T20D device...");
            uartLoRaWanDevice = new E32_868T20D(Device, Device.SerialPortNames.Com1, Device.Pins.D10, Device.Pins.D11, Device.Pins.D14);
            uartLoRaWanDevice.DataReceived += DataReceived;

            msg = new StringBuilder();
            Console.WriteLine("");
        }

        private void DataReceived(object sender, byte[] receivedData)
        {
            Console.WriteLine($"Data received: {BitConverter.ToString(receivedData)}");
            msg.Append(Encoding.ASCII.GetString(receivedData, 0, receivedData.Length));
            try
            {
                var resultString = Regex.Match(msg.ToString(), @"\d{5}T\d{2}\.\d{2}").Value;
                if (resultString != null && resultString.Length > 0)
                {
                    Console.WriteLine($"Raw Message: {resultString}");
                    Console.WriteLine($"Decoded Message: Message n. {resultString.Substring(0,5)}, temperature {resultString.Substring(6)} Celsius");
                    Console.WriteLine();
                    msg.Remove(0, resultString.Length);
                }
            }
            catch (ArgumentException ex)
            {
                // Syntax error in the regular expression
                Console.WriteLine(ex.Message);
            }
        }

        private async Task ManageLoRaWanCommunications()
        {
            // Request version
            Console.WriteLine("Request version...");
            await uartLoRaWanDevice.RequestVersion().ConfigureAwait(false);

            Console.WriteLine($"LoRaWAN mdule version: {uartLoRaWanDevice.LoRaWanVersion}");
            Console.WriteLine();

            // Request configuration parameters
            Console.WriteLine("Request configuration parameters...");
            await uartLoRaWanDevice.RequestConfigParams().ConfigureAwait(false);

            Console.WriteLine("Actual device configuration:");
            Console.WriteLine("");
            ShowConfiguration();

            Thread.Sleep(1000);

            // Set configuration parameters

            //Console.WriteLine("Set configuration parameters...");
            //await uartLoRaWanDevice.SetParameters(saveParamsOnPwrDown: false,
            //                                      moduleAddress: 0x000,
            //                                      dataFrame: UartLoRaWanDataFrame.EightNoneOne,
            //                                      baudRate: 9600,
            //                                      airDataRate: 2400,
            //                                      channelMHz: 868,
            //                                      transmissionType: UartLoRaWanTransmissionType.Trasparent,
            //                                      ioDriveMode: UartLoRaWanIoDriveMode.PushPullUp,
            //                                      wakeUpTimeMs: 250,
            //                                      forwardErrorCorrection: true,
            //                                      transmissionPowerDb: 20).ConfigureAwait(false);

            //Console.WriteLine("New device configuration:");
            //Console.WriteLine("");
            //ShowConfiguration();
        }

        private void ShowConfiguration()
        {
            Console.WriteLine($"Parameters row data: {BitConverter.ToString(uartLoRaWanDevice.LoRaWanParameters)}");
            Console.WriteLine("Decoded parameters:");
            Console.WriteLine("-------------------------------------------");
            Console.WriteLine($"Save parameters on power down: {uartLoRaWanDevice.SaveParamsOnPwrDown}");
            Console.WriteLine($"Module address: {uartLoRaWanDevice.DataFrame}");
            Console.WriteLine($"Data frame: {uartLoRaWanDevice.DataFrame}");
            Console.WriteLine($"Baud rate: {uartLoRaWanDevice.BaudRate}");
            Console.WriteLine($"Ait data rate: {uartLoRaWanDevice.AirDataRate}");
            Console.WriteLine($"Channel: {uartLoRaWanDevice.ChannelMHz} MHz");
            Console.WriteLine($"Transmission type: {uartLoRaWanDevice.TransmissionType}");
            Console.WriteLine($"I/O drive mode: {uartLoRaWanDevice.IoDriveMode}");
            Console.WriteLine($"Wake up time: {uartLoRaWanDevice.WakeUpTimeMs} ms");
            Console.WriteLine($"FEC (forward error correction): {(uartLoRaWanDevice.ForwardErrorCorrection ? "enabled" : "disabled")}");
            Console.WriteLine($"Transmission power: {uartLoRaWanDevice.TransmissionPowerDb} db");
            Console.WriteLine("");
        }
    }
}
