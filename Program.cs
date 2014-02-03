//#define HTDU21D_ADDRESS 0x40  //Unshifted 7-bit I2C address for the sensor
//#define TRIGGER_TEMP_MEASURE_HOLD  0xE3
//#define TRIGGER_HUMD_MEASURE_HOLD  0xE5
//#define TRIGGER_TEMP_MEASURE_NOHOLD  0xF3
//#define TRIGGER_HUMD_MEASURE_NOHOLD  0xF5
//#define WRITE_USER_REG  0xE6
//#define READ_USER_REG  0xE7
//#define SOFT_RESET  0xFE


using System;
using System.IO;
using System.Threading;
using Microsoft.SPOT;
using Microsoft.SPOT.IO;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.NetduinoPlus;
using i2cSensors;
using NetduinoPlusTesting;


namespace WeatherBoard
{
    public class Program
    {
        
        private static HTU21D_HumiditySensor HSensor;
        private static BMP085_BaroSensor BSensor;
             


        public static void Main()
        {
            // Local barometric pressure for alititude calculation correction. Weather dependent. 
            float AltCorrection = 102700;  

            // Create a new I2C bus instance at startup.
            I2CBus i2cBus = I2CBus.GetInstance();
            HSensor = new HTU21D_HumiditySensor(0x40);
            BSensor = new BMP085_BaroSensor(0x77);
                        
            while (true)
            {
                           
                float temp = HSensor.ReadTemp();
                float rh = HSensor.ReadHumidity();                              
                
                long Btemp = BSensor.bmp085GetTemperature();
                long pressure = BSensor.bmp085GetPressure();
                
                // calculate altitude 
                double altitude = 44330 * (1 - System.Math.Pow(((float)pressure / AltCorrection), (1 / 5.255)));
                

                Debug.Print("Temp Sensor1, " + temp.ToString() + ", Relative Humidity, " + rh.ToString() + ", Barometric Pressure, " + pressure.ToString() + ", Temp Sensor2, " + Btemp.ToString() + ", Altitude(m), " + altitude.ToString());

                // Write line of weather data to .csv file. 
                using (var filestream = new FileStream(@"\SD\weatherlog.csv", FileMode.Append))
                {
                    StreamWriter writer = new StreamWriter(filestream);

                    writer.WriteLine("Temp Sensor1, " + temp.ToString() + ", Relative Humidity, " + rh.ToString() + ", Barometric Pressure, " + pressure.ToString() + ", Temp Sensor2, " + Btemp.ToString() + ", Altitude(m), " + altitude.ToString());
                
                    writer.Close();
                }                
                
                Thread.Sleep(1000);
                
           }
        }

    }
}
