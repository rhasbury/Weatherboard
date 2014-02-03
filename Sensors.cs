using System;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.Hardware;
using NetduinoPlusTesting;

namespace i2cSensors
{
   
    /// This class contains methods for reading and writing to the HTU21D humidity sensor. 
    public class HTU21D_HumiditySensor
    {
        private I2CDevice.Configuration _slaveConfig;
        private const int TransactionTimeout = 1000; // ms
        private const byte ClockRateKHz = 250;
        public byte Address { get; private set; }
 
        
        public HTU21D_HumiditySensor(byte address)
        {
            Address = address;
            _slaveConfig = new I2CDevice.Configuration(address, ClockRateKHz);
        }


        // ReadTemp() reads and caclulates the temperature from the device and returns the temperatrure as a float. 
        public float ReadTemp()
        {

            I2CBus.GetInstance().Write(_slaveConfig, new byte[1] { 0xF3 }, TransactionTimeout); // Send the "Trigger Temperature Measurement 0xF3 No Hold master" command to device. 
            Thread.Sleep(60);                                                                    // Wait for measurement to excecute. (This could possibly be shortened). 

            byte[] data = new byte[2];
            I2CBus.GetInstance().Read(_slaveConfig, data, TransactionTimeout);                  // Read two byte raw temperature into data[]

            uint rawTemperature = ((uint)data[0] << 8) | (uint)data[1];                         // Place MSB and LSB in rawtemperature. 
            rawTemperature &= 0xFFFC;                                                           // Mask off last two status bits. 
            
            float tempTemperature = rawTemperature / (float)65536; 
            float realTemperature = (float)-46.85 + ((float)175.72 * tempTemperature);          // Temperature calculations as indicated by the datasheet. 

            return realTemperature;                                                             // Return temperature. 
        }

        // ReadHumidity() reads and calculates humidity from the device and returns as a float. 
        public float ReadHumidity()
        {

            I2CBus.GetInstance().Write(_slaveConfig, new byte[1] { 0xF5 }, TransactionTimeout); // Send the "Trigger Humidity Measurement 0xF5 No Hold master" command to device. 
            Thread.Sleep(60);                                                                   // Wait for measurement to excecute. (This could possibly be shortened). 
           
            byte[] data = new byte[2];
            I2CBus.GetInstance().Read(_slaveConfig, data, TransactionTimeout);                  // Read two byte raw humidity into data[]

            uint rawRH = ((uint)data[0] << 8) | (uint)data[1];                                  // Place MSB and LSB in rawRH.
            rawRH &= 0xFFFC;                                                                    // Mask off last two status bits. 
            float tempRH = rawRH / (float)65536;
            float rh = -6 + (125 * tempRH);                                                     // Humidity calcuations as indicated by the datasheet. 

            return rh;                                                                          // Return relative humidity
        }

    }




    /// This class contains methods for reading and writing to the BMP085 barometric pressure sensor.   
    public class BMP085_BaroSensor
    {
        private static I2CDevice.Configuration _slaveConfig;
        private const int TransactionTimeout = 1000; // ms
        private const byte ClockRateKHz = 250;
        public byte Address { get; private set; }
        
        private const byte OSS = 0;                         // Oversampling setting. 
        
        // Static locations for calibration data to be read from the sensor.           
        public static short ac1;
        public static short ac2;
        public static short ac3;
        public static ushort ac4;
        public static ushort ac5;
        public static ushort ac6;
        public static short b1;
        public static short b2;
        public static short mb;
        public static short mc;
        public static short md;
        public static long b5;

        // temporary variables. 
        public static short temperature;
        public static long pressure;

        // Use these for altitude conversions
        const float p0 = 101325;     // Pressure at sea level (Pa)
        public static float altitude;

         
        public BMP085_BaroSensor(byte address)
        {
            // initialize i2c device. 
            Address = address;
            _slaveConfig = new I2CDevice.Configuration(address, ClockRateKHz);
            
            // Pull calbration data from sensor and store in variables. 
            GetInitialCalibration();                                       
        }


        
        public int ReadSomethingFromBaroRegister(byte register)
        {
            // get MSB and LSB result
            byte[] data = new byte[2];
            I2CBus.GetInstance().ReadRegister(_slaveConfig, register, data, TransactionTimeout);
            int returndata = ((int)data[0] << 8) | (int)data[1];            
            return returndata;
        }
        
        
                
        public static long bmp085ReadUP()
            {              
              long up = 0;  
              // send pressure read command
              I2CBus.GetInstance().Write(_slaveConfig, new byte[2] { 0xF4, (0x34 + (OSS<<6))}, TransactionTimeout);
              
              // Wait for conversion, delay time dependent on OSS              
              Thread.Sleep(5 + (3<<OSS));
  
              // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)

              byte[] data = new byte[3];
              I2CBus.GetInstance().ReadRegister(_slaveConfig, 0xF6, data, TransactionTimeout);
              //Debug.Print("MSB " + data[0].ToString() + " LSB " + data[1].ToString() + " XLSB " + data[2].ToString());
 
              up = ((data[0] << 16) | (data[1] << 8) | data[2]) >> (8-OSS);
  
              return up;
            }



        public static long bmp085ReadUT()
        {
           
            long ut = 0;
            
            I2CBus.GetInstance().Write(_slaveConfig, new byte[2] { 0xF4, 0x2E }, TransactionTimeout);

            // Wait for conversion, delay time dependent on OSS              
            Thread.Sleep(10);

            // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
            byte[] data = new byte[2];
            I2CBus.GetInstance().ReadRegister(_slaveConfig, 0xF6, data, TransactionTimeout);
                        
            ut = (((long)data[0] << 8) | (long)data[1])  ;

            return ut;
        }
        
        



        public void GetInitialCalibration()
        {
            ac1 = (short)ReadSomethingFromBaroRegister(0xAA);
            ac2 = (short)ReadSomethingFromBaroRegister(0xAC);
            ac3 = (short)ReadSomethingFromBaroRegister(0xAE);
            ac4 = (ushort)ReadSomethingFromBaroRegister(0xB0);
            ac5 = (ushort)ReadSomethingFromBaroRegister(0xB2);
            ac6 = (ushort)ReadSomethingFromBaroRegister(0xB4);
            b1 = (short)ReadSomethingFromBaroRegister(0xB6);
            b2 = (short)ReadSomethingFromBaroRegister(0xB8);
            mb = (short)ReadSomethingFromBaroRegister(0xBA);
            mc = (short)ReadSomethingFromBaroRegister(0xBC);
            md = (short)ReadSomethingFromBaroRegister(0xBE);


            // Test data from datasheet for testing calculations. 
            //ac1 = 408;
            //ac2 = -72;
            //ac3 = -14383;
            //ac4 = 32741;
            //ac5 = 32757;
            //ac6 = 23153;
            //b1 = 6190;
            //b2 = 4;
            //mb = -32768;
            //mc = -8711;
            //md = 2868;
        }

        public long bmp085GetTemperature()
        {

            long ut = bmp085ReadUT();
            long x1, x2;

            x1 = (ut - ac6) * ac5 >> 15;
            x2 = (mc << 11) / (x1 + md);
            b5 = (x1 + x2);
            return ((b5 + 8) >> 4);
        }

        public long bmp085GetPressure()
        {

            long up = bmp085ReadUP();

            long x1, x2, x3, b3, b6, p;
            ulong b4, b7;

            b6 = b5 - 4000;
            // Calculate B3
            x1 = (b2 * (b6 * b6 >> 12)) >> 11;            
            x2 = (ac2 * b6) >> 11;            
            x3 = x1 + x2;            
            b3 = ((ac1 * 4 + x3) << OSS +2) >> 4;
          

            // Calculate B4
            x1 = (ac3 * b6) >> 13;
            x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
            x3 = ((x1 + x2) + 2) >> 2;
            b4 = (ulong)(ac4 * (x3 + 32768)) >> 15;

            b7 = (ulong)((up - b3) * (50000 >> OSS));
            if (b7 < 0x80000000)
                p = (long)((b7 << 1) / b4);
            else
                p = (long)(b7 / b4) << 1;

            x1 = (p >> 8) * (p >> 8);
            x1 = (x1 * 3038) >> 16;
            x2 = (-7357 * p) >> 16;
            p += (x1 + x2 + 3791) >> 4;

            return p;
        }
    }
}
