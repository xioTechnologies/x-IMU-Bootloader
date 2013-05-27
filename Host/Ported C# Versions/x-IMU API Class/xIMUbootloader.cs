using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.IO.Ports;
using System.Threading;

namespace xIMU_API
{
    /// <summary>
    /// x-IMU bootloader class used to update firmware.
    /// </summary>
    public class xIMUbootloader
    {
        #region Variables

        private const int PM_SIZE = 1536;
        private const int EE_SIZE = 128;
        private const int CM_SIZE = 8;
        private const int BUFFER_SIZE = 4096;
        private const int PM30F_ROW_SIZE = 32;
        private const int PM33F_ROW_SIZE = 64 * 8;
        private const int EE30F_ROW_SIZE = 16;

        private const byte COMMAND_NACK = 0x00;
        private const byte COMMAND_ACK = 0x01;
        private const byte COMMAND_READ_PM = 0x02;
        private const byte COMMAND_WRITE_PM = 0x03;
        private const byte COMMAND_READ_EE = 0x04;
        private const byte COMMAND_WRITE_EE = 0x05;
        private const byte COMMAND_READ_CM = 0x06;
        private const byte COMMAND_WRITE_CM = 0x07;
        private const byte COMMAND_RESET = 0x08;
        private const byte COMMAND_READ_ID = 0x09;

        private enum eFamily
        {
            dsPIC30F,
            dsPIC33F,
            PIC24H,
            PIC24F
        };

        private struct sDevice
        {
            public string pName;
            public ushort Id;
            public ushort ProcessId;
            public eFamily Family;

            public sDevice(string _n, ushort _id, ushort _pid, eFamily _fam) { pName = _n; Id = _id; ProcessId = _pid; Family = _fam; }
        }

        #region Long array

        private sDevice[] Device = new sDevice[] {
            new sDevice ("dsPIC30F2010",      0x040, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F2011",      0x0C0, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F2011",      0x240, 1, eFamily.dsPIC30F),
	        new sDevice ("dsPIC30F2012",      0x0C2, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F2012",      0x241, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F3010",      0x1C0, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F3011",      0x1C1, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F3012",      0x0C1, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F3013",      0x0C3, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F3014",      0x160, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F4011",      0x101, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F4012",      0x100, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F4013",      0x141, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F5011",      0x080, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F5013",      0x081, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F5015",      0x200, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F5016",      0x201, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6010",      0x188, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6010A",     0x281, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6011",      0x192, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6011A",     0x2C0, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6012",      0x193, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6012A",     0x2C2, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6013",      0x197, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6013A",     0x2C1, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6014",      0x198, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6014A",     0x2C3, 1, eFamily.dsPIC30F),
            new sDevice ("dsPIC30F6015",      0x280, 1, eFamily.dsPIC30F),

            new sDevice ("dsPIC33FJ64GP206",  0xC1, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64GP306",  0xCD, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64GP310",  0xCF, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64GP706",  0xD5, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64GP708",  0xD6, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64GP710",  0xD7, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128GP206", 0xD9, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128GP306", 0xE5, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128GP310", 0xE7, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128GP706", 0xED, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128GP708", 0xEE, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128GP710", 0xEF, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ256GP506", 0xF5, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ256GP510", 0xF7, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ256GP710", 0xFF, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64MC506",  0x89, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64MC508",  0x8A, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64MC510",  0x8B, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64MC706",  0x91, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64MC710",  0x97, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128MC506", 0xA1, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128MC510", 0xA3, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128MC706", 0xA9, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128MC708", 0xAE, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128MC710", 0xAF, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ256MC510", 0xB7, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ256MC710", 0xBF, 3, eFamily.dsPIC33F),

            new sDevice ("dsPIC33FJ12GP201", 0x802, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ12GP202", 0x803, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ12MC201", 0x800, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ12MC202", 0x801, 3, eFamily.dsPIC33F),

            new sDevice ("dsPIC33FJ32GP204", 0xF0F, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ32GP202", 0xF0D, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ16GP304", 0xF07, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ32MC204", 0xF0B, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ32MC202", 0xF09, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ16MC304", 0xF03, 3, eFamily.dsPIC33F),

            new sDevice ("dsPIC33FJ128GP804", 0x62F, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128GP802", 0x62D, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128GP204", 0x627, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128GP202", 0x625, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64GP804",  0x61F, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64GP802",  0x61D, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64GP204",  0x617, 3, eFamily.dsPIC33F),
	        new sDevice ("dsPIC33FJ64GP202",  0x615, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ32GP304",  0x607, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ32GP302",  0x605, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128MC804", 0x62B, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128MC802", 0x629, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128MC204", 0x623, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ128MC202", 0x621, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64MC804",  0x61B, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64MC802",  0x619, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64MC204",  0x613, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ64MC202",  0x611, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ32MC304",  0x603, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ32MC302",  0x601, 3, eFamily.dsPIC33F),

            new sDevice ("dsPIC33FJ06GS101",  0xC00, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ06GS102",  0xC01, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ06GS202",  0xC02, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ16GS402",  0xC04, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ16GS404",  0xC06, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ16GS502",  0xC03, 3, eFamily.dsPIC33F),
            new sDevice ("dsPIC33FJ16GS504",  0xC05, 3, eFamily.dsPIC33F),

            new sDevice ("PIC24HJ64GP206",    0x41, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ64GP210",    0x47, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ64GP506",    0x49, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ64GP510",    0x4B, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ128GP206",   0x5D, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ128GP210",   0x5F, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ128GP306",   0x65, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ128GP310",   0x67, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ128GP506",   0x61, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ128GP510",   0x63, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ256GP206",   0x71, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ256GP210",   0x73, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ256GP610",   0x7B, 3, eFamily.PIC24H),

            new sDevice ("PIC24HJ12GP201", 0x80A, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ12GP202", 0x80B, 3, eFamily.PIC24H),

            new sDevice ("PIC24HJ32GP204", 0xF1F, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ32GP202", 0xF1D, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ16GP304", 0xF17, 3, eFamily.PIC24H),

            new sDevice ("PIC24HJ128GP504", 0x67F, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ128GP502", 0x67D, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ128GP204", 0x667, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ128GP202", 0x665, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ64GP504",  0x677, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ64GP502",  0x675, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ64GP204",  0x657, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ64GP202",  0x655, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ32GP304",  0x647, 3, eFamily.PIC24H),
            new sDevice ("PIC24HJ32GP302",  0x645, 3, eFamily.PIC24H),

            new sDevice ("PIC24FJ64GA006",    0x405, 3, eFamily.PIC24F),
            new sDevice ("PIC24FJ64GA008",    0x408, 3, eFamily.PIC24F),
            new sDevice ("PIC24FJ64GA010",    0x40B, 3, eFamily.PIC24F),
            new sDevice ("PIC24FJ96GA006",    0x406, 3, eFamily.PIC24F),
            new sDevice ("PIC24FJ96GA008",    0x409, 3, eFamily.PIC24F),
            new sDevice ("PIC24FJ96GA010",    0x40C, 3, eFamily.PIC24F),
            new sDevice ("PIC24FJ128GA006",   0x407, 3, eFamily.PIC24F),
            new sDevice ("PIC24FJ128GA008",   0x40A, 3, eFamily.PIC24F),
            new sDevice ("PIC24FJ128GA010",   0x40D, 3, eFamily.PIC24F),
        };

        #endregion

        #region Nested memRow class

        private class memRow
        {
            #region Varables

            public enum eType
            {
                Program = 0,
                EEprom,
                Config
            };

            byte[]      m_pBuffer;
            uint        m_Address;
            bool        m_bEmpty;
            eType       m_eType;
            ushort[]    m_Data;
            int         m_RowNumber;
            eFamily     m_eFamily;
            int         m_RowSize;

            #endregion

            #region Constructor

            /// <summary>
            /// Initializes a new instance of the <see cref="T:memRow"/> class.
            /// </summary>
            public memRow(eType Type, uint StartAddress, int RowNumber, eFamily Family)
            {
                m_Data = new ushort[PM33F_ROW_SIZE * 2];

                int Size = 0;
                m_RowNumber = RowNumber;
                m_eFamily = Family;
                m_eType = Type;
                m_bEmpty = true;

                if (m_eType == eType.Program)
                {
                    if (m_eFamily == eFamily.dsPIC30F)
                    {
                        m_RowSize = PM30F_ROW_SIZE;
                    }
                    else
                    {
                        m_RowSize = PM33F_ROW_SIZE;
                    }
                }
                else
                {
                    m_RowSize = EE30F_ROW_SIZE;
                }

                if (m_eType == eType.Program)
                {
                    Size = m_RowSize * 3;
                    m_Address = (uint)(StartAddress + RowNumber * m_RowSize * 2);
                }
                if (m_eType == eType.EEprom)
                {
                    Size = m_RowSize * 2;
                    m_Address = (uint)(StartAddress + RowNumber * m_RowSize * 2);
                }
                if (m_eType == eType.Config)
                {
                    Size = 3;
                    m_Address = (uint)(StartAddress + RowNumber * 2);
                }

                m_pBuffer = new byte[Size];

                for (int i = 0; i < (PM33F_ROW_SIZE * 2); i++) m_Data[i] = 0xFFFF;
            }

            #endregion

            #region Methods

            public bool InsertData(uint Address, string pData)
            {
                if (Address < m_Address)
                {
                    return false;
                }

                if ((m_eType == eType.Program) && (Address >= (m_Address + m_RowSize * 2)))
                {
                    return false;
                }

                if ((m_eType == eType.EEprom) && (Address >= (m_Address + m_RowSize * 2)))
                {
                    return false;
                }

                if ((m_eType == eType.Config) && (Address >= (m_Address + 2)))
                {
                    return false;
                }

                m_bEmpty = false;

                m_Data[Address - m_Address] = (ushort)((CharToUShort(pData[0]) << 12) | (CharToUShort(pData[1]) << 8) | (CharToUShort(pData[2]) << 4) | (CharToUShort(pData[3])));

                return true;
            }

            private ushort CharToUShort(char p)
            {
                if ((((byte)p) > 0x2F) && (((byte)p) < 0x3A)) return (ushort)((byte)p - 0x30);      //results in 0-9
                if ((((byte)p) > 0x40) && (((byte)p) < 0x47)) return (ushort)((byte)p - 55);        //results in a-f
                if ((((byte)p) > 0x60) && (((byte)p) < 0x67)) return (ushort)((byte)p - 87);        //results in a-f
                throw new Exception("Cannot convert " + p.ToString() + " into a number");           //if goes wrong, do f
            }

            public void FormatData()
            {
                if (m_bEmpty) return;

                if (m_eType == eType.Program)
                {
                    for (int Count = 0; Count < m_RowSize; Count++)
                    {
                        m_pBuffer[0 + Count * 3] = (byte)((m_Data[Count * 2] >> 8) & 0xFF);
                        m_pBuffer[1 + Count * 3] = (byte)((m_Data[Count * 2]) & 0xFF);
                        m_pBuffer[2 + Count * 3] = (byte)((m_Data[Count * 2 + 1] >> 8) & 0xFF);
                    }
                }
                else if (m_eType == eType.Config)
                {
                    m_pBuffer[0] = (byte)((m_Data[0] >> 8) & 0xFF);
                    m_pBuffer[1] = (byte)((m_Data[0]) & 0xFF);
                    m_pBuffer[2] = (byte)((m_Data[1] >> 8) & 0xFF);
                }
                else
                {
                    for (int Count = 0; Count < m_RowSize; Count++)
                    {
                        m_pBuffer[0 + Count * 2] = (byte)((m_Data[Count * 2] >> 8) & 0xFF);
                        m_pBuffer[1 + Count * 2] = (byte)((m_Data[Count * 2]) & 0xFF);
                    }
                }
            }

            public void SendData(ref SerialPort pComDev)
            {
                byte[] Buffer = new byte[] { 0, 0, 0, 0 };

                if ((m_bEmpty) && (m_eType != eType.Config)) return;

                while (Buffer[0] != COMMAND_ACK)
                {
                    if (m_eType == eType.Program)
                    {
                        Buffer[0] = COMMAND_WRITE_PM;
                        Buffer[1] = (byte)((m_Address) & 0xFF);
                        Buffer[2] = (byte)((m_Address >> 8) & 0xFF);
                        Buffer[3] = (byte)((m_Address >> 16) & 0xFF);

                        pComDev.Write(Buffer, 0, 4);
                        pComDev.Write(m_pBuffer, 0, m_RowSize * 3);
                    }
                    else if (m_eType == eType.EEprom)
                    {
                        Buffer[0] = COMMAND_WRITE_EE;
                        Buffer[1] = (byte)((m_Address) & 0xFF);
                        Buffer[2] = (byte)((m_Address >> 8) & 0xFF);
                        Buffer[3] = (byte)((m_Address >> 16) & 0xFF);

                        pComDev.Write(Buffer, 0, 4);
                        pComDev.Write(m_pBuffer, 0, m_RowSize * 2);
                    }
                    else if ((m_eType == eType.Config) && (m_RowNumber == 0))
                    {
                        Buffer[0] = COMMAND_WRITE_CM;
                        Buffer[1] = (byte)((m_bEmpty) ? 0x01 : 0x00);
                        Buffer[2] = m_pBuffer[0];
                        Buffer[3] = m_pBuffer[1];

                        pComDev.Write(Buffer, 0, 4);
                    }
                    else if ((m_eType == eType.Config) && (m_RowNumber != 0))
                    {
                        if ((m_eFamily == eFamily.dsPIC30F) && (m_RowNumber == 7)) return;

                        Buffer[0] = (byte)((m_bEmpty) ? 0x01 : 0x00);
                        Buffer[1] = m_pBuffer[0];
                        Buffer[2] = m_pBuffer[1];

                        pComDev.Write(Buffer, 0, 3);
                    }
                    else
                    {
                        throw new Exception("Unknown memory type specified in source file");
                    }
                    try
                    {
                        Buffer[0] = (byte)(pComDev.ReadByte() & 0xFF);
                    }
                    catch (Exception ex)
                    {
                        throw ex;
                    }
                }
            }

            #endregion
        }

        #endregion

        private SerialPort serialPort;
        private string portName;
        private StreamReader streamReader;
        private string privSourceFile;

        private int LoadRow = 0;

        private memRow[] ppMemory;

        #endregion

        #region Properties

        /// <summary>
        /// Gets or sets the name of the serial port.
        /// </summary>
        public string PortName
        {
            get
            {
                return portName;
            }
            set
            {
                serialPort.PortName = value;
                portName = serialPort.PortName;
            }
        }

        /// <summary>
        /// Gets a value indicated the open or closed value of the serial port.
        /// </summary>
        public bool IsOpen
        {
            get
            {
                return serialPort.IsOpen;
            }
        }

        /// <summary>
        /// Gets or sets the full path of the .hex source file.
        /// </summary>
        public string SourceFile
        {
            get
            {
                return privSourceFile;
            }
            set
            {
                privSourceFile = value;
            }
        }

        #endregion

        #region Constructors

        /// <summary>
        /// Initializes a new instance of the <see cref="T:xIMUbootloader"/> class.
        /// </summary>
        public xIMUbootloader()
            : this("COM1", "")
        {
        }

        /// <summary>
        /// Initializes a new instance of the <see cref="T:xIMUbootloader"/> class.
        /// </summary>
        /// <param name="portName">
        /// Name of the port the x-IMU appears as (for example, COM1).
        /// </param>
        /// <param name="sourceFile">
        /// Full path of the firmware .hex source file.
        /// </param>
        public xIMUbootloader(string portName, string sourceFile)
        {
            serialPort = new SerialPort(portName, 115200, Parity.None, 8, StopBits.One);
            serialPort.Handshake = Handshake.RequestToSend;
            serialPort.ReceivedBytesThreshold = 1;
            //serialPort.ReadTimeout = 500; //long, so obvious if going wrong
            //serialPort.WriteTimeout = 500;
            privSourceFile = sourceFile;
        }

        #endregion

        #region Public methods

        /// <summary>
        /// Uploads firmware to x-IMU.  x-IMU must be first be put in bootloader mode.
        /// </summary>
        public void Upload()
        {
            uint ExtAddr = 0;
            byte[] Buffer = new byte[BUFFER_SIZE];

            #region Open source file

            try
            {
                streamReader = new StreamReader(privSourceFile);
            }
            catch (Exception ex)
            {
                throw new Exception(ex.Message);
            }

            #endregion

            #region Open serial port

            try
            {
                serialPort.Open();
                serialPort.DiscardInBuffer();
            }
            catch (Exception ex)
            {
                streamReader.Close();
                throw new Exception(ex.Message);
            }

            #endregion

            #region Check target family ID

            eFamily Family;
            try
            {
                Family = ReadID();
            }
            catch (Exception ex)
            {
                streamReader.Close();
                serialPort.Close();
                throw new Exception("x-io board not responding on " + serialPort.PortName, ex);
            }

            #endregion

            #region Read source file

            try
            {
                ppMemory = new memRow[PM_SIZE + EE_SIZE + CM_SIZE];
                for (int Row = 0; Row < PM_SIZE; Row++)
                {
                    ppMemory[Row] = new memRow(memRow.eType.Program, 0, Row, eFamily.dsPIC33F);
                }
                for (int Row = 0; Row < EE_SIZE; Row++)
                {
                    ppMemory[Row + PM_SIZE] = new memRow(memRow.eType.EEprom, 0x7FF000, Row, eFamily.dsPIC33F);
                }
                for (int Row = 0; Row < CM_SIZE; Row++)
                {
                    ppMemory[Row + PM_SIZE + EE_SIZE] = new memRow(memRow.eType.Config, 0xF80000, Row, eFamily.dsPIC33F);
                }
                while (!streamReader.EndOfStream)             //read the whole hex file, parse and store
                {
                    uint ByteCount = 0;
                    uint Address = 0;
                    uint RecordType = 0;

                    string aLine = streamReader.ReadLine();

                    ByteCount = (uint)(((CharToUShort(aLine[1]) << 4) & 0xF0) | (CharToUShort(aLine[2]) & 0xF));
                    Address = (uint)(((CharToUShort(aLine[3]) << 12) & 0xF000) | ((CharToUShort(aLine[4]) << 8) & 0xF00) | ((CharToUShort(aLine[5]) << 4) & 0xF0) | (CharToUShort(aLine[6]) & 0xF));
                    RecordType = (uint)(((CharToUShort(aLine[7]) << 4) & 0xF0) | (CharToUShort(aLine[8]) & 0xF));

                    if (RecordType == 0)
                    {
                        Address = (uint)(Address + ExtAddr) / 2;
                        for (int CharCount = 0; CharCount < ByteCount * 2; CharCount += 4, Address++)
                        {
                            bool bInserted = false;
                            for (int Row = 0; Row < (PM_SIZE + EE_SIZE + CM_SIZE); Row++)
                            {
                                if ((bInserted = ppMemory[Row].InsertData(Address, aLine.Substring(9 + CharCount)) == true))
                                {
                                    break;
                                }
                            }
                            if (bInserted != true)
                            {
                                serialPort.Close();
                                throw new Exception("Bad hex file: Address out of range");
                            }
                        }

                    }
                    else if (RecordType == 1)
                    {
                    }
                    else if (RecordType == 4)
                    {
                        ExtAddr = (uint)(((CharToUShort(aLine[9]) << 12) & 0xF000) | ((CharToUShort(aLine[10]) << 8) & 0xF00) | ((CharToUShort(aLine[11]) << 4) & 0xF0) | (CharToUShort(aLine[12]) & 0xF));
                        ExtAddr <<= 16;
                    }
                    else
                    {
                        //Error
                    }
                }
            }
            catch (Exception ex)
            {
                serialPort.Close();
                //ProgressForm.Close();
                throw ex;
            }

            try
            {
                streamReader.Close();
            }
            catch { }

            #endregion

            //Preserve first two locations for bootloader
            {
                string Data;
                int RowSize;
                int i = 0;

                if (Family == eFamily.dsPIC30F)
                {
                    RowSize = PM30F_ROW_SIZE;
                }
                else RowSize = PM33F_ROW_SIZE;

                Buffer[0] = COMMAND_READ_PM;
                Buffer[1] = 0x00;
                Buffer[2] = 0x00;
                Buffer[3] = 0x00;

                serialPort.Write(Buffer, 0, 4);
                //Thread.Sleep(100);

                //UpdateForm("Reading target..", 0); Application.DoEvents();

                try
                {
                    for (i = 0; i < ((RowSize * 3) - 500); i++)
                        Buffer[i] = (byte)serialPort.ReadByte();
                }
                catch (Exception ex)
                {
                    serialPort.Close();
                    //ProgressForm.Close();
                    //throw new Exception("Charcount: " + i.ToString() + " out of " + Convert.ToString((RowSize * 3)) + " -- " + ex.Message);
                    throw new Exception(ex.Message + " on char: " + i.ToString() + " out of " + Convert.ToString((RowSize * 3)));
                }
                //serialPort.Read(Buffer,0,RowSize * 3);

                Data = String.Format("{0:x2}{1:x2}{2:x2}00{3:x2}{4:x2}{5:x2}00", Buffer[2], Buffer[1], Buffer[0], Buffer[5], Buffer[4], Buffer[3]);

                ppMemory[0].InsertData(0x000000, Data);
                ppMemory[0].InsertData(0x000001, Data.Substring(4));
                ppMemory[0].InsertData(0x000002, Data.Substring(8));
                ppMemory[0].InsertData(0x000003, Data.Substring(12));

                serialPort.DiscardInBuffer();
            }

            //UpdateForm("Formatting data..", 0);
            for (int Row = 0; Row < (PM_SIZE + EE_SIZE + CM_SIZE); Row++) ppMemory[Row].FormatData();

            //((ProgressBar)ProgressForm.Controls[1]).Maximum = PM_SIZE + EE_SIZE + CM_SIZE;
            //UpdateForm("Uploading firmware...", 0);


            /////////////////////////////////////////////////////
            while (true)
            {
                //Application.DoEvents();                         //just in case something else happened, let the program deal with messages

                try
                {
                    ppMemory[LoadRow++].SendData(ref serialPort);
                }
                catch (Exception ex)
                {
                    serialPort.Close();
                    //ProgressForm.Close();
                    throw ex;
                }

                //UpdateForm(String.Format("Uploading firmware... {0:00.00}% done", ((double)LoadRow / (double)(PM_SIZE + EE_SIZE + CM_SIZE)) * 100), LoadRow);

                if (LoadRow >= (PM_SIZE + EE_SIZE + CM_SIZE))
                {
                    //UpdateForm("Firmware update complete", LoadRow);

                    byte[] rsBuffer = new byte[] { COMMAND_RESET };
                    serialPort.Write(rsBuffer, 0, 1);

                    //Application.DoEvents();
                    Thread.Sleep(300);
                    serialPort.Close();
                    //ProgressForm.Close();
                    //onBootloadFinished();
                    break;
                }
            }
        }

        #endregion

        #region Private methods

        //Necessary functions for bootloader
        private eFamily ReadID()
        {
            byte[] Buffer = new byte[BUFFER_SIZE];
            ushort DeviceID = 0;
            ushort ProcessID = 0;

            Buffer[0] = COMMAND_READ_ID;

            serialPort.Write(Buffer, 0, 1);
            Thread.Sleep(50);
            serialPort.Read(Buffer, 0, 8);

            DeviceID = (ushort)(((((uint)Buffer[1]) << 8) & 0xFF00) | (((uint)Buffer[0]) & 0x00FF));
            ProcessID = (ushort)((((uint)Buffer[5]) >> 4) & 0x0F);


            int Count = 0;
            bool bDeviceFound = false;

            while (bDeviceFound != true)
            {
                if (Count >= Device.Length) break;

                if ((Device[Count].Id == DeviceID) && (Device[Count].ProcessId == ProcessID))
                {
                    bDeviceFound = true;
                    break;
                }
                Count++;
            }

            if (!bDeviceFound) throw new Exception("Could not find a valid DeviceID");

            //UpdateForm(String.Format("Found " + Device[Count].pName + " (ID: 0x{0:x4})", DeviceID), 95);
            Thread.Sleep(200);

            return (Device[Count].Family);
        }

        private ushort CharToUShort(char p)
        {
            if ((((byte)p) > 0x2F) && (((byte)p) < 0x3A)) return (ushort)((byte)p - 0x30);      //results in 0-9
            if ((((byte)p) > 0x40) && (((byte)p) < 0x47)) return (ushort)((byte)p - 55);        //results in a-f
            if ((((byte)p) > 0x60) && (((byte)p) < 0x67)) return (ushort)((byte)p - 87);        //results in a-f
            throw new Exception("Cannot convert " + p.ToString() + " into a number");
        }

        #endregion
    }
}
