using Meadow.Devices;
using Meadow.Hardware;
using System;
using System.Text;
using System.Threading.Tasks;

namespace TPCWare.LoRaWAN
{
    class E32_868T20D: IUartLoRaWan
    {
        // The model E32_868T20D use RF from 862 MHz to 893 MHz that is the range allowed in EU coutries.
        private const int DEVICE_MIN_MHZ = 862; 
        private const int DEVICE_MAX_MHZ = 893;

        // SerialPort baud rate and data frame to connect to the device
        private const int BAUDRATE = 9600;
        private const int DATABITS = 8;
        private const Parity PARITY = Parity.None;
        private const StopBits STOPBITS = StopBits.One;

        private const int MAXIMUM_BUFFER_SIZE = 512;

        private readonly int[] BAUD_RATES = new int[] { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };
        private readonly int[] AIR_DATA_RATES = new int[] { 300, 1200, 2400, 4800, 9600, 19200, 19200, 19200 };
        private readonly int[] WAKE_UP_TIME_MS = new int[] { 250, 500, 750, 1000, 1250, 1500, 1750, 2000 };
        private readonly int[] TRANSMISSION_POWER_DB = new int[] { 20, 17, 14, 10 }; // For E32-TTL-100, E32-TTL-100S1, E32-T100S2

        // Other settings for other devices
        // private readonly int[] TRANSMISSION_POWER_DB = new int[] { 27, 24, 21, 18 }; // For E32-TTL-500
        // private readonly int[] TRANSMISSION_POWER_DB = new int[] { 30, 27, 24, 21 }; // For E32-TTL-1W, E32 (433T30S), E32(868T30S) E32 (915T30S)

        private readonly static object serialReadLock = new Object();
        private readonly byte[] readBuffer = new byte[MAXIMUM_BUFFER_SIZE];

        private readonly IDigitalOutputPort m0, m1;
        private readonly IDigitalInputPort aux;
        private readonly ISerialPort serialPort;

        private UartLoRaWanOpMode opMode = UartLoRaWanOpMode.Normal;
        public UartLoRaWanOpMode OpMode
        {
            get
            {
                return opMode;
            }
            set
            {
                if (opMode != value)
                {
                    opMode = value;
                    switch (opMode)
                    {
                        case UartLoRaWanOpMode.Normal:
                            m0.State = false;
                            m1.State = false;
                            break;
                        case UartLoRaWanOpMode.WakeUp:
                            m0.State = true;
                            m1.State = false;
                            break;
                        case UartLoRaWanOpMode.PowerSaving:
                            m0.State = false;
                            m1.State = true;
                            break;
                        case UartLoRaWanOpMode.Sleep:
                            m0.State = true;
                            m1.State = true;
                            break;
                        default:
                            m0.State = false;
                            m1.State = false;
                            break;
                    }
                }
            }
        }

        private readonly byte[] loRaWanVersion = new byte[4];
        public string LoRaWanVersion => loRaWanVersion[2].ToString() + "." + loRaWanVersion[3].ToString();

        private readonly byte[] loRaWanParameters = new byte[6];
        public byte[] LoRaWanParameters => loRaWanParameters;

        public bool SaveParamsOnPwrDown => loRaWanParameters[0] == 0xC0;
        public int ModuleAddress => 256 * loRaWanParameters[1] + loRaWanParameters[2];
        public UartLoRaWanDataFrame DataFrame
        {
            get
            {
                var dataFrame = loRaWanParameters[3] >> 6;
                if (dataFrame == 0 || dataFrame == 4) return UartLoRaWanDataFrame.EightNoneOne;
                if (dataFrame == 1) return UartLoRaWanDataFrame.EightOddOne;
                return UartLoRaWanDataFrame.EightEvenOne;
            }
        }
        public int BaudRate => BAUD_RATES[(loRaWanParameters[3] & 0b00111000) >> 3];
        public int AirDataRate => AIR_DATA_RATES[loRaWanParameters[3] & 0b00000111];
        public int ChannelMHz => DEVICE_MIN_MHZ + (loRaWanParameters[4] & 0b00011111);
        public UartLoRaWanTransmissionType TransmissionType => (UartLoRaWanTransmissionType)(loRaWanParameters[5] >> 7);
        public UartLoRaWanIoDriveMode IoDriveMode => (UartLoRaWanIoDriveMode)((loRaWanParameters[5] & 0b01000000) >> 6);
        public int WakeUpTimeMs => WAKE_UP_TIME_MS[(loRaWanParameters[5] & 0b00111000) >> 3];
        public bool ForwardErrorCorrection => (loRaWanParameters[5] & 0b00000100) > 0;
        public int TransmissionPowerDb => TRANSMISSION_POWER_DB[loRaWanParameters[5] & 0b00000011];

        public event EventHandler<byte[]> DataReceived;

        public E32_868T20D(IMeadowDevice device,
                           SerialPortName serialPortName,
                           IPin pinM0,
                           IPin pinM1,
                           IPin pinAux,
                           UartLoRaWanOpMode opMode = UartLoRaWanOpMode.Normal)
        {
            serialPort = device.CreateSerialPort(serialPortName, BAUDRATE, DATABITS, PARITY, STOPBITS);

            serialPort.DataReceived += SerialPortDataReceived;
            serialPort.Open();

            m0 = device.CreateDigitalOutputPort(pinM0, false);
            m1 = device.CreateDigitalOutputPort(pinM1, false);
            aux = device.CreateDigitalInputPort(pinAux);
            OpMode = opMode;
        }

        private void OpenSerialPort()
        {
            if (serialPort is null)
                throw new Exception("Unable to open the serial port (reference is null).");
            else
            {
                if (!serialPort.IsOpen)
                {
                    serialPort.Open();
                }
            }
        }

        private void CloseSerialPort()
        {
            if (serialPort is null)
                throw new Exception("Unable to close the serial port (reference is null).");
            else
            {
                if (serialPort.IsOpen)
                {
                    serialPort.Close();
                }
            }
        }

        private void ChangeSerialPortBaudRate(int baudRate)
        {
            if (serialPort is null)
                throw new Exception("Unable to change the baud rate of the serial port (reference is null).");

            // Check baud rate
            int baudRateIndex = Array.IndexOf(BAUD_RATES, baudRate);
            if (baudRateIndex < 0)
                throw new Exception($"The requested baud rate of {baudRate} bps is not available on this device.");

            CloseSerialPort();
            serialPort.BaudRate = baudRate;
            OpenSerialPort();
        }

        private void SerialPortDataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            // Ignore event if Operational Mode is sleep (parameters handling is made directly by related methods)
            if (OpMode != UartLoRaWanOpMode.Sleep)
            {
                lock (serialReadLock)
                {
                    int amount = serialPort.ReadAll(readBuffer);

                    if (amount > 0)
                    {
                        var receivedData = new byte[amount];
                        Buffer.BlockCopy(readBuffer, 0, receivedData, 0, amount);
                        DataReceived?.Invoke(this, receivedData);
                    }
                }
            }

            // Console.WriteLine($"SerialPort {serialPort.PortName} DataReceived: {BitConverter.ToString(receivedData,0,receivedDataOffset)}");
        }

        public async Task WriteToSerialPortAsync(string utf8Data)
        {
            await WriteToSerialPortAsync(Encoding.UTF8.GetBytes(utf8Data)).ConfigureAwait(false);
        }

        public async Task WriteToSerialPortAsync(byte[] bytes)
        {
            if (serialPort is null)
                throw new Exception("Unable to write to the serial port (reference is null).");
            else if (!serialPort.IsOpen)
                throw new Exception("The serial port {serialPort.PortName} is not open.");
            else
            {
                // Console.WriteLine($"Writing {bytes.Length} bytes: {BitConverter.ToString(bytes)}");
                int offset = 0;
                while (bytes.Length - offset > 0)
                {
                    if (aux.State)
                    {
                        // Even if the aux signal is HIGH, we need to wait 2 ms to be sure the LoRaWan device is ready to get from the UART the message to send wirelessly
                        await Task.Delay(2).ConfigureAwait(false);

                        if (bytes.Length - offset > MAXIMUM_BUFFER_SIZE)
                        {
                            _ = serialPort.Write(bytes, offset, MAXIMUM_BUFFER_SIZE);
                            offset += MAXIMUM_BUFFER_SIZE;
                        }
                        else
                        {
                            _ = serialPort.Write(bytes, offset, bytes.Length - offset);
                            offset = bytes.Length;
                        }
                    }
                    else
                    {
                        // If the aux signal is LOW, the LoRaWan device is not ready, maybe because resetting or because still sending a previous message
                        await Task.Delay(300).ConfigureAwait(false);
                    }
                }
            }
        }

        public async Task ResetModule()
        {
            // To get and set parameters, the device as to be on Sleep OpMode
            var previousOpMode = OpMode;
            OpMode = UartLoRaWanOpMode.Sleep;

            // Set the baudrate to 9600 because when in sleep mode the device is set to 9600 baud rate and 8N1 data frame
            ChangeSerialPortBaudRate(9600);

            // To reset device we need to send hex code 'C1C1C1'
            await WriteToSerialPortAsync(new byte[] { 0XC4, 0XC4, 0XC4 }).ConfigureAwait(false);

            // Wait one second
            await Task.Delay(1000).ConfigureAwait(false);

            // After resetting we need to get current parameters
            await WriteToSerialPortAsync(new byte[] { 0XC1, 0XC1, 0XC1 }).ConfigureAwait(false);

            // Wait one second
            await Task.Delay(1000).ConfigureAwait(false);

            // Read parameters
            lock (serialReadLock)
            {
                int amount = serialPort.Read(loRaWanParameters, 0, 6);
            }

            // Change the serial port baud rate accordingly to current parametes
            ChangeSerialPortBaudRate(BaudRate);

            // Back to previous operating mode
            OpMode = previousOpMode;
        }

        public async Task RequestVersion()
        {
            // To get and set parameters, the device as to be on Sleep OpMode
            var previousOpMode = OpMode;
            OpMode = UartLoRaWanOpMode.Sleep;

            // Set the baudrate to 9600 because when in sleep mode the device is set to 9600 baud rate and 8N1 data frame
            ChangeSerialPortBaudRate(9600);

            // To get actual parameters we need to send hex code 'C1C1C1'
            await WriteToSerialPortAsync(new byte[] { 0XC3, 0XC3, 0XC3 }).ConfigureAwait(false);

            // Wait one second
            await Task.Delay(1000).ConfigureAwait(false);

            // Read version
            lock (serialReadLock)
            {
                int amount = serialPort.Read(loRaWanVersion, 0, 4);
            }

            // Change the serial port baud rate accordingly to current parametes
            ChangeSerialPortBaudRate(BaudRate);

            // Back to previous operating mode
            OpMode = previousOpMode;
        }

        public async Task RequestConfigParams()
        {
            // To get and set parameters, the device as to be on Sleep OpMode
            var previousOpMode = OpMode;
            OpMode = UartLoRaWanOpMode.Sleep;

            // Set the baudrate to 9600 because when in sleep mode the device is set to 9600 baud rate and 8N1 data frame
            ChangeSerialPortBaudRate(9600);

            // To get actual parameters we need to send hex code 'C1C1C1'
            await WriteToSerialPortAsync(new byte[] { 0XC1, 0XC1, 0XC1 }).ConfigureAwait(false);

            // Wait one second
            await Task.Delay(1000).ConfigureAwait(false);

            // Read parameters
            lock (serialReadLock)
            {
                int amount = serialPort.Read(loRaWanParameters, 0, 6);
            }

            // Change the serial port baud rate accordingly to current parametes
            ChangeSerialPortBaudRate(BaudRate);

            // Back to previous operating mode
            OpMode = previousOpMode;
        }

        /// <summary>
        /// Set the device parameters
        /// </summary>
        /// <param name="saveParamsOnPwrDown">If true, parameters will be persisted.</param>
        /// <param name="moduleAddress">From exadecimal 0000 to FFFF.</param>
        /// <param name="dataFrame">Although those setting are all valid for the device, the only implemented and available setting is 8N1.</param>
        /// <param name="baudRate">Choose from 1200, 2400, 4800, 9600, 19200, 38400, 57600 and 115200 bps.</param>
        /// <param name="airDataRate">Choose from 300, 1200, 2400, 4800, 9600 and 19200 bps.</param>
        /// <param name="channelMHz">Choose from 862 to 893 MHz.</param>
        /// <param name="transmissionType">If Fixed, mode, the first three bytes of each user's data frame represent high low address and channel.</param>
        /// <param name="ioDriveMode">Open drain mode may need external pull up resistor.</param>
        /// <param name="wakeUpTimeMs">Choose from 250, 500, 750, 1000, 1250, 1500, 1750 and 2000 ms.</param>
        /// <param name="forwardErrorCorrection">Disabling FEC can improve transmission speed but at the cost of reduced distance.</param>
        /// <param name="transmissionPowerDb">Choose from 20, 17, 14 and 10 db.</param>
        /// <returns></returns>
        public async Task SetParameters(bool saveParamsOnPwrDown = false,
                                        int moduleAddress = 0x0000,
                                        UartLoRaWanDataFrame dataFrame = UartLoRaWanDataFrame.EightNoneOne,
                                        int baudRate = 9600,
                                        int airDataRate = 2400,
                                        int channelMHz = (DEVICE_MIN_MHZ + 6),
                                        UartLoRaWanTransmissionType transmissionType = UartLoRaWanTransmissionType.Trasparent,
                                        UartLoRaWanIoDriveMode ioDriveMode = UartLoRaWanIoDriveMode.PushPullUp,
                                        int wakeUpTimeMs = 250,
                                        bool forwardErrorCorrection = true,
                                        int transmissionPowerDb = 20)
        {
            // Check data frame
            if (dataFrame != UartLoRaWanDataFrame.EightNoneOne)
                throw new Exception($"Although data frame {dataFrame} is valid for the device, the only implemented and available setting is 8N1");

            // Check baud rate
            int baudRateIndex = Array.IndexOf(BAUD_RATES, baudRate);
            if (baudRateIndex < 0)
                throw new Exception($"The requested baud rate of {baudRate} bps is not available on this device.");

            // Check air data rate
            int airDataRateIndex = Array.IndexOf(AIR_DATA_RATES, airDataRate);
            if (airDataRateIndex < 0)
                throw new Exception($"The requested air data rate of {airDataRate} bps is not available on this device.");

            // Check channel
            if (channelMHz < DEVICE_MIN_MHZ || channelMHz > DEVICE_MAX_MHZ)
                throw new Exception($"The requested channel of {channelMHz} MHz is outside the device range of {DEVICE_MIN_MHZ}-{DEVICE_MAX_MHZ} MHz.");

            // Check wake up time
            int wakeUpTimeMsIndex = Array.IndexOf(WAKE_UP_TIME_MS, wakeUpTimeMs);
            if (wakeUpTimeMsIndex < 0)
                throw new Exception($"The requested awake up time of {wakeUpTimeMs} ms is not available on this device.");

            // Check transmission power
            int transmissionPowerDbIndex = Array.IndexOf(TRANSMISSION_POWER_DB, transmissionPowerDb);
            if (transmissionPowerDbIndex < 0)
                throw new Exception($"The requested transmission power of {transmissionPowerDb} db is not available on this device.");

            byte[] setLoRaWanParameters = new byte[6];

            setLoRaWanParameters[0] = saveParamsOnPwrDown ? (byte)0xC0 : (byte)0xC2;
            setLoRaWanParameters[1] = (byte)(moduleAddress >> 8);
            setLoRaWanParameters[2] = (byte)(moduleAddress);
            setLoRaWanParameters[3] = (byte)((int)dataFrame * 64 + baudRateIndex * 8 + airDataRateIndex);
            setLoRaWanParameters[4] = (byte)(channelMHz - DEVICE_MIN_MHZ);
            setLoRaWanParameters[5] = (byte)((int)transmissionType * 128 + (int)ioDriveMode * 64 + wakeUpTimeMsIndex * 8 + Convert.ToInt32(forwardErrorCorrection) * 4 + transmissionPowerDbIndex);

            Console.WriteLine($"Calculated parameters row data: {BitConverter.ToString(setLoRaWanParameters)}");

            // To get and set parameters, the device as to be on Sleep OpMode
            var previousOpMode = OpMode;
            OpMode = UartLoRaWanOpMode.Sleep;

            // Set the baudrate to 9600 because when in sleep mode the device is set to 9600 baud rate and 8N1 data frame
            ChangeSerialPortBaudRate(9600);

            // Set parameters
            await WriteToSerialPortAsync(setLoRaWanParameters).ConfigureAwait(false);

            // Wait one second
            await Task.Delay(1000).ConfigureAwait(false);

            // Read parameters
            lock (serialReadLock)
            {
                int amount = serialPort.Read(loRaWanParameters, 0, 6);
            }

            // Change the serial port baud rate accordingly to current parametes
            ChangeSerialPortBaudRate(BaudRate);

            // Back to previous operating mode
            OpMode = previousOpMode;
        }
    }

    public enum UartLoRaWanOpMode
    {
        Normal = 0,
        WakeUp = 1,
        PowerSaving = 2,
        Sleep = 3
    }

    // Although those setting are all valid for the device, the only implemented and available setting is 8N1
    public enum UartLoRaWanDataFrame
    {
        EightNoneOne = 0,
        EightOddOne = 1,
        EightEvenOne = 2
    }

    public enum UartLoRaWanTransmissionType
    {
        Trasparent = 0,
        Fixed = 1
    }

    public enum UartLoRaWanIoDriveMode
    {
        // TXD AUX open collector outputs, RXD open collector input
        OpenCollector = 0,
        //TXD and AUX push pull outputs, RXD pull up input
        PushPullUp = 1
    }
}
