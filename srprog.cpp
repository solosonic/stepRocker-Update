
/*
 *  StepRocker firmware programming utility
 *  see http://www.motioncontrol-community.org
 *
 *  Build with e.g.:
 *      g++ -pipe -O2 -Wall -o srprog srprog.cpp
 *
 *  Copyright (c) 2012, Wolfgang Hoffmann <woho@woho.de>
 *
 *  Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee
 *  is hereby granted, provided that the above copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS
 *  SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE
 *  AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 *  NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 *  OF THIS SOFTWARE.
 */

#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <string.h>
#include <string>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

// configuration
#define CONFIG_SPECIFIC_PORT_FOR_BOOTLOADER_REQUEST
#define CONFIG_LX200_BOOTLOADER_REQUEST

// helper for outputing a formated decimal or hex number
struct OutputNum
{
    int m_nValue;
    int m_nWidth;
    bool m_bHex;
    const char *m_szPrefix;
    OutputNum(int nValue, int nWidth = 0, bool bHex = false, const char *szPrefix = "") : m_nValue(nValue), m_nWidth(nWidth), m_bHex(bHex), m_szPrefix(szPrefix) {}
};
inline OutputNum outputDec2(int nValue)
{
    return OutputNum(nValue, 2);
}
inline OutputNum outputHex2(int nValue)
{
    return OutputNum(nValue, 2, true);
}
inline OutputNum outputHex8(int nValue)
{
    return OutputNum(nValue, 8, true, "0x");
}
template<typename _CharT, typename _Traits> inline std::basic_ostream<_CharT, _Traits> &operator<<(std::basic_ostream<_CharT, _Traits> &oStream, OutputNum oVal)
{
    if (oVal.m_bHex)
        oStream << oVal.m_szPrefix << std::setfill('0') << std::setw(oVal.m_nWidth) << std::hex << oVal.m_nValue << std::dec << std::setw(0);
    else
        oStream << oVal.m_szPrefix << std::setfill('0') << std::setw(oVal.m_nWidth) << oVal.m_nValue << std::setw(0);
    return oStream;
}

// class for reading intel hex image file
class IntelHexImage
{
public:
    IntelHexImage(const char *szFileName, size_t nAddrMax) :
        m_bParserFoundEof(false),
        m_nAddrMax(nAddrMax),
        m_nBeg(1),
        m_nEnd(0),
        m_nAddrSeg(0),
        m_nAddrExt(0)
    {
        std::ifstream oFile(szFileName, std::ifstream::in);
        std::cout << "parsing HEX file " << szFileName << std::endl;
        while (oFile.good())
        {
            std::string strLine;
            std::getline(oFile, strLine);
            if (!parseLine(strLine))
                break;
            if (m_bParserFoundEof)
                break;
        }
        //std::cout << std::endl;
    }
    bool valid() const { return (m_bParserFoundEof && (m_nBeg < m_nEnd)); }
    int beg() const { return m_nBeg; }
    int end() const { return m_nEnd; }
    std::vector<unsigned char> data() const { return m_aData; }
    unsigned int checksum() const
    {
        unsigned int nChecksum = 0;
        for (size_t nAddr = m_nBeg; nAddr < m_nEnd; nAddr++)
            nChecksum += m_aData[nAddr];
        return nChecksum;
    }
private:
    bool parseLine(const std::string &strLine)
    {
        std::stringstream oLine(strLine);

        // Intel HEX format: see http://en.wikipedia.org/wiki/Intel_HEX

        // line must start with ':'
        if (!(oLine.good() && (oLine.get() == ':') && oLine.good()))
        {
            std::cerr << "missing ':' at start of line: " << strLine << std::endl;
            return false;
        }

        // convert text line from ascii hexdigits to byte vector
        std::vector<unsigned char> aField;
        while (true)
        {
            char szDigits[3];
            oLine >> szDigits[0] >> szDigits[1];
            szDigits[2] = '\0';
            if (!oLine.good())
                break;
            int nVal;
            std::stringstream(szDigits) >> std::hex >> nVal;
            aField.push_back(nVal);
        }

        // split bytearray into byte count, address, record type, data and checksum
        if (aField.size() < 1)
        {
            std::cerr << "line too short: " << strLine << std::endl;
            return false;
        }
        size_t nByteCount = aField[0];
        if (aField.size() != 1 + 2 + 1 + nByteCount + 1)
        {
            std::cerr << "line too short/long (" << aField.size() << " vs. " << 1 + 2 + 1 + nByteCount + 1 << "): " << strLine << " ";
            for (size_t nCnt = 0; nCnt < aField.size(); nCnt++)
                std::cerr << std::hex << (int)aField[nCnt];
            std::cerr << std::endl;
            return false;
        }
        int nChecksum = 0;
        for (size_t nCnt = 0; nCnt < aField.size(); nCnt++)
            nChecksum += aField[nCnt];
        if ((nChecksum & 0xff) != 0)
        {
            std::cerr << "wrong checksum for line: " << strLine << std::endl;
            return false;
        }
        size_t nAddress = (((size_t)aField[1] << 8) & 0xff00) | ((size_t)aField[2] & 0xff); // big endian
        int nRecordType = aField[3];

        // add record to flash sram image
        switch (nRecordType)
        {
        case 0: // Data Record
            nAddress += m_nAddrSeg + m_nAddrExt;
            if (nAddress >= m_nAddrMax)
            {
                std::cerr << "address " << outputHex8(nAddress) << ' '
                    << "beyond limit " << outputHex8(m_nAddrMax) << std::endl;
                return false;
            }
            if ((m_nBeg > nAddress) || (m_nBeg > m_nEnd))
                m_nBeg = nAddress;
            if (m_nEnd < nAddress + nByteCount)
                m_nEnd = nAddress + nByteCount;
            if (m_aData.size() < m_nEnd)
                m_aData.resize(m_nEnd, 0xff);
            for (size_t nCnt = 0; nCnt < nByteCount; nCnt++)
                m_aData[nAddress + nCnt] = aField[4 + nCnt];
            //std::cout << ".";
            break;
        case 1: // End Of File record
            m_bParserFoundEof = true;
            break;
        case 2: // Extended Segment Address Record
            if (!((nAddress == 0) && (nByteCount == 2) && ((aField[4 + 1] & 0x0f) == 0)))
            {
                std::cerr << "wrong Extended Segment Address Record: " << strLine << std::endl;
                return false;
            }
            m_nAddrSeg = ((((size_t)aField[4 + 0] << 8) & 0xff00) | ((size_t)aField[4 + 1] & 0xff)) << 4; // big endian
            break;
        case 4: // Extended Linear Address Record
            if (!((nAddress == 0) && (nByteCount == 2)))
            {
                std::cerr << "wrong Extended Linear Address Record: " << strLine << std::endl;
                return false;
            }
            m_nAddrExt = ((((size_t)aField[4 + 0] << 8) & 0xff00) | ((size_t)aField[4 + 1] & 0xff)) << 16; // big endian
            break;
        case 3: // Start Segment Address Record: ignore
        case 5: // Start Linear Address Record: ignore
            break;
        default:
            std::cerr << "unknown record type: " << strLine << std::endl;
            return false;
        }
        return true;
    }
private:
    bool m_bParserFoundEof;
    size_t m_nAddrMax;
    size_t m_nBeg;
    size_t m_nEnd;
    std::vector<unsigned char> m_aData;
    size_t m_nAddrSeg;
    size_t m_nAddrExt;
};

// TMCL command opcodes
enum TmclOpcode {
    TMCL_GetVersion = 136,      // type: 0 = as string, 1 = as binary, motor: 0 (not used), value: 0 (not used)
    TMCL_BootEraseAll = 200,    // type: 0 (not used), motor: 0 (not used), value: 0 (not used)
    TMCL_BootWriteBuffer = 201, // type: index (0 .. 255), motor: 0 (not used), value: 4 bytes of data (one 32-bit word)
    TMCL_BootWritePage = 202,   // type: 0 (not used), motor: 0 (not used), value: physical memory address (within page)
    TMCL_BootGetChecksum = 203, // type: 0 (not used), motor: 0 (not used), value: last address
    TMCL_BootReadMemory = 204,  // type: 0 (not used), motor: 0 (not used), value: address (multiple of 4)
    TMCL_BootStartAppl = 205,   // type: 0 (not used), motor: 0 (not used), value: 0 (not used)
    TMCL_BootGetInfo = 206,     // type: 0 = page size, 1 = application start address, 2 = total flash size, motor: 0 (not used), value: 0 (not used)
    TMCL_Boot = 0xf2            // type: 0x81 (magic value), motor: 0x92 (magic value), value: 0xa3b4c5d6 (magic value)
};

// TMCL command container
class TmclCmd
{
public:
    TmclCmd(unsigned char nAddress, unsigned char nOpcode, unsigned char nType, unsigned char nMotor, int nValue)
    {
        m_aCmd[0] = nAddress;
        m_aCmd[1] = nOpcode;
        m_aCmd[2] = nType;
        m_aCmd[3] = nMotor;
        m_aCmd[4] = (nValue >> 24) & 0xff;
        m_aCmd[5] = (nValue >> 16) & 0xff;
        m_aCmd[6] = (nValue >> 8) & 0xff;
        m_aCmd[7] = nValue & 0xff;
        unsigned char nCheckSum = 0;
        for (int i = 0; i < 8; i++)
            nCheckSum += m_aCmd[i];
        m_aCmd[8] = nCheckSum;
    }
    TmclCmd(unsigned char nAddress, unsigned char nOpcode, unsigned char nType, unsigned char nMotor, const std::vector<unsigned char> &aValue, size_t nOffs)
    {
        m_aCmd[0] = nAddress;
        m_aCmd[1] = nOpcode;
        m_aCmd[2] = nType;
        m_aCmd[3] = nMotor;
        m_aCmd[4] = (nOffs + 3 < aValue.size())? aValue[nOffs + 3]: 0;
        m_aCmd[5] = (nOffs + 2 < aValue.size())? aValue[nOffs + 2]: 0;
        m_aCmd[6] = (nOffs + 1 < aValue.size())? aValue[nOffs + 1]: 0;
        m_aCmd[7] = (nOffs + 0 < aValue.size())? aValue[nOffs + 0]: 0;
        unsigned char nCheckSum = 0;
        for (int i = 0; i < 8; i++)
            nCheckSum += m_aCmd[i];
        m_aCmd[8] = nCheckSum;
    }
    const unsigned char *raw() const { return m_aCmd; }
private:
    unsigned char m_aCmd[9];
};

// TMCL answer container
class TmclAns
{
public:
    TmclAns() : m_bValid(false) {}
    TmclAns(const unsigned char *aCmd, const unsigned char *aAns) :
        m_bValid(false)
    {
        // copy answer
        for (int i = 0; i < 9; i++)
            m_aAns[i] = aAns[i];
        // calculate checksum
        unsigned char nCheckSum = 0;
        for (int i = 0; i < 8; i++)
            nCheckSum += m_aAns[i];
        m_bValid = (
            (m_aAns[1] == aCmd[0]) &&   // matching command and answer address
            (m_aAns[2] == 100) &&       // status: 100 == OK
            (m_aAns[3] == aCmd[1]) &&   // matching command and answer instruction
            (m_aAns[8] == nCheckSum));  // correct checksum
    }
    bool valid() const { return m_bValid; }
    const unsigned char *raw() const { return m_aAns; }
    int value() const
    {
        int nValue =
            (((int)m_aAns[4] & 0xff) << 24) |
            (((int)m_aAns[5] & 0xff) << 16) |
            (((int)m_aAns[6] & 0xff) << 8) |
            (((int)m_aAns[7] & 0xff));
        return nValue;
    }
private:
    bool m_bValid;
    unsigned char m_aAns[9];
};

// class for TMCL communication with StepRocker device over serial port
class TmclDevice
{
public:
    // Serial device access: see http://www.tldp.org/HOWTO/Serial-Programming-HOWTO/x115.html
    TmclDevice(bool bDebug) :
        m_szPortName(""),
        m_nFd(-1),
        m_bDebug(bDebug)
    {
    }
    ~TmclDevice()
    {
        close();
    }
    bool open(const char *szPortName)
    {
        if (m_nFd >= 0)
            return false;

        m_szPortName = szPortName;
        m_nFd = ::open(m_szPortName, O_RDWR | O_NOCTTY);
        if (m_nFd < 0)
        {
            std::cerr << "open(" << m_szPortName << "): ";
            perror("");
            return false;
        }
        std::cout << "open " << m_szPortName << ": " << m_nFd << std::endl;

        tcgetattr(m_nFd, &m_oOldTio);   // save current port settings

        bzero(&m_oNewTio, sizeof(m_oNewTio));
        m_oNewTio.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
        m_oNewTio.c_iflag = IGNPAR;
        m_oNewTio.c_oflag = 0;
        m_oNewTio.c_lflag = 0;          // set input mode (non-canonical, no echo,...)
        m_oNewTio.c_cc[VTIME] = 1;      // inter-character timer: 1 * 100 ms
        m_oNewTio.c_cc[VMIN] = 0;       // don't block if no chars received

        tcflush(m_nFd, TCIFLUSH);
        tcsetattr(m_nFd, TCSANOW, &m_oNewTio);

        return true;
    }
    void close()
    {
        if (m_nFd < 0)
            return;
        tcsetattr(m_nFd, TCSANOW, &m_oOldTio);
        ::close(m_nFd);
        m_nFd = -1;
        m_szPortName = "";
    }
    void flush()
    {
        if (m_nFd < 0)
            return;
        tcflush(m_nFd, TCIFLUSH);
    }
    bool send(const unsigned char *pData, int nLen)
    {
        if (m_bDebug)
            std::cout << "send " << hex(pData, nLen) << std::endl;
        int nCnt = 0;
        while (nCnt < nLen)
        {
            int nRet = write(m_nFd, pData + nCnt, nLen - nCnt);
            if (nRet < 0)
            {
                std::cerr << "write(" << m_szPortName << "): ";
                perror("");
                return false;
            }
            nCnt += nRet;
        }
        return true;
    }
    bool recv(unsigned char *pData, int nLen, int nTimeout = 1000)
    {
        int nCnt = 0;
        while (nCnt < nLen)
        {
            int nRet = read(m_nFd, pData + nCnt, nLen - nCnt);
            if (nRet < 0)
            {
                std::cerr << "read(" << m_szPortName << "): ";
                perror("");
                return false;
            }
            else if (nRet == 0)
            {
                nTimeout -= 100;
                if (nTimeout < 0)
                    return false;
            }
            nCnt += nRet;
        }
        if (m_bDebug)
            std::cout << "recv " << hex(pData, nLen) << std::endl;
        return true;
    }
    TmclAns send(const TmclCmd &oCmd, int nTimeout = 1000)
    {
        const unsigned char *aCmd = oCmd.raw();
        if (!send(aCmd, 9))
            return TmclAns();
        unsigned char aAns[9];
        if (!recv(aAns, 9, nTimeout))
            return TmclAns();
        return TmclAns(aCmd, aAns);
    }
    std::string hex(const unsigned char *aData, int nNum)
    {
        std::stringstream oData;
        for (int nByte = 0; nByte < nNum; nByte++)
            oData << outputHex2(aData[nByte]) << ' ';
        oData << '"';
        for (int nByte = 0; nByte < nNum; nByte++)
            oData << (isprint(aData[nByte])? (char)(aData[nByte]): '.');
        oData << '"';
        return oData.str();
    }

private:
    const char *m_szPortName;
    int m_nFd;
    bool m_bDebug;
    termios m_oOldTio;
    termios m_oNewTio;
};

int main(int argc, char *argv[])
{
    // parse commandline arguments
    bool bShowUsage = false;
    const char *szFileName = 0;
    const char *szPortProgram = "/dev/ttyACM0";
    const char *szPortRequest = 0;
    bool bTryTmcl = true;
    bool bTryLx200 = false;
    bool bDebug = false;
    for (int nArg = 1; nArg < argc; nArg++)
    {
        std::string strArg = argv[nArg];
        if ((strArg == "-?") || (strArg == "-h") || (strArg == "--help"))
            bShowUsage = true;
        else if ((strArg == "-P") || (strArg == "--port"))
        {
            nArg += 1;
            if (nArg < argc)
                szPortProgram = argv[nArg];
            else
                bShowUsage = true;
        }
#ifdef CONFIG_SPECIFIC_PORT_FOR_BOOTLOADER_REQUEST
        else if ((strArg == "-B") || (strArg == "--port-bootrequest"))
        {
            nArg += 1;
            if (nArg < argc)
                szPortRequest = argv[nArg];
            else
                bShowUsage = true;
        }
#endif // CONFIG_SPECIFIC_PORT_FOR_BOOTLOADER_REQUEST
#ifdef CONFIG_LX200_BOOTLOADER_REQUEST
        else if ((strArg == "-L") || (strArg == "--lx200"))
            bTryLx200 = true;
#endif // CONFIG_LX200_BOOTLOADER_REQUEST
        else if ((strArg == "-d") || (strArg == "--debug"))
            bDebug = true;
        else if (nArg == argc - 1)
            szFileName = argv[nArg];
        else
            bShowUsage = true;
    }
    if (szPortRequest == 0)
        szPortRequest = szPortProgram;
    if (bShowUsage || (szFileName == 0))
    {
        std::cout << "usage:" << std::endl;
        std::cout << "    " << argv[0] << " [options] <hexfile name>" << std::endl;
        std::cout << "options:" << std::endl;
        std::cout << "    -P, --port <serial port>" << std::endl;
        std::cout << "        serial port for programming the device. <serial port> typically is:" << std::endl;
        std::cout << "        /dev/ttyACM0 when programming via the StepRocker onboard USB port" << std::endl;
        std::cout << "        /dev/ttyUSB0 when programming via a FTDI based RS485 adaptor" << std::endl;
        std::cout << "        default: /dev/ttyACM0" << std::endl;
#ifdef CONFIG_SPECIFIC_PORT_FOR_BOOTLOADER_REQUEST
        std::cout << "    -B, --port-bootrequest <serial port>" << std::endl;
        std::cout << "        serial port for requesting the bootloader" << std::endl;
        std::cout << "        use this when running an open source based firmware that has no" << std::endl;
        std::cout << "        USB support, but want to benefit from faster programming via the" << std::endl;
        std::cout << "        onboard USB port." << std::endl;
        std::cout << "        default: value of --port" << std::endl;
#endif // CONFIG_SPECIFIC_PORT_FOR_BOOTLOADER_REQUEST
#ifdef CONFIG_LX200_BOOTLOADER_REQUEST
        std::cout << "    -L, --lx200" << std::endl;
        std::cout << "        try LX200 based protocol (astronomical mount control firmware)" << std::endl;
        std::cout << "        when rebooting into bootloader" << std::endl;
        std::cout << "        default: false" << std::endl;
#endif // CONFIG_LX200_BOOTLOADER_REQUEST
        std::cout << "    -d, --debug" << std::endl;
        std::cout << "        print sent/received data on standard out for debugging communication" << std::endl;
        std::cout << "        default: false" << std::endl;
        std::cout << "examples:" << std::endl;
        std::cout << "    program Trinamic TMCL firmware V1.03:" << std::endl;
        std::cout << "        " << argv[0] << " ~/Downloads/trinamic/stepRockerTMCL_V103.hex" << std::endl;
        std::cout << "    program custom mini-TMCL based firmware, using RS485 for bootloader request" << std::endl;
        std::cout << "    and onboard USB for programming:" << std::endl;
        std::cout << "        " << argv[0] << " -B /dev/ttyUSB0 -P /dev/ttyACM0 ~/value/devel/astro/qenqenet/qenqenet/bin/sr/ROM_RUN/qenqenet.hex" << std::endl;
        std::cout << std::endl;
        return 0;
    }

    // read hex image
    IntelHexImage oImage(szFileName, 0x00040000);
    if (!oImage.valid())
    {
        std::cout << "failed to parse image " << szFileName << std::endl;
        return -1;
    }
    int nChecksum = oImage.checksum();
    std::cout << "image " << szFileName << ": beg " << outputHex8(oImage.beg()) << ", end " << outputHex8(oImage.end()) << ", checksum " << outputHex8(nChecksum) << std::endl;

    // open target device
    TmclDevice oTarget(bDebug);
    TmclAns oAns;
    if (!oTarget.open(szPortRequest))
        return -1;

    // try to identify target device and reboot into bootloader
    bool bKnownDevice = false;
    if (bTryTmcl)
    {
        // TMCL based firmware
        std::cout << "trying TMCL protocol to identify device ..." << std::endl;
        oAns = oTarget.send(TmclCmd(1, TMCL_GetVersion, 1, 0, 0)); // get firmware version (type = binary)
        if (oAns.valid())
        {
            unsigned int nVersion = oAns.value();
            std::cout << "found TMCL-based module " << (nVersion >> 16) << ", firmware version " << ((nVersion >> 8) & 0xff) << "." << outputDec2(nVersion & 0xff) << std::endl;
            std::cout << "requesting bootloader ..." << std::endl;
            oTarget.flush();
            oTarget.send(TmclCmd(1, TMCL_Boot, 0x81, 0x92, 0xa3b4c5d6), 0); // reboot into bootloader (no answer!)
            bKnownDevice = true;
        }
    }
    if (bTryLx200 && (!bKnownDevice))
    {
        // LX200 based firmware (astronomical mount control)
        // see http://www.meade.com/support/TelescopeProtocol_2010-10.pdf
        std::cout << "trying LX200 protocol to identify device ..." << std::endl;
        static unsigned char aCmdAlignmentQuery[] = "\x06";
        static unsigned char aCmdFirmwareDownloadRequest[] = "\x04";
        unsigned char aAns[1];
        if (oTarget.send(aCmdAlignmentQuery, 1) &&
            oTarget.recv(aAns, 1) &&
            (aAns[0] == 'P'))
        {
            std::cout << "found LX200-based module." << std::endl;
            std::cout << "requesting bootloader ..." << std::endl;
            oTarget.flush();
            oTarget.send(aCmdFirmwareDownloadRequest, 1);
            bKnownDevice = true;
        }
    }
    if (!bKnownDevice)
    {
        std::cout << "failed to identify device." << std::endl;
        return -1;
    }

    // To enjoy faster programming via onboard USB when coming from a
    // mini-TMCL based firmware that has no USB support, we need to request
    // the bootloader via RS485 and program the firmware via USB.
    // However, it seems that when requesting the bootloader via RS485, the bootloader
    // only accepts programming commands from RS485, but not from the onboard USB port.
    // Workaround that by enforcing a cold boot into the bootloader:
    // erase the flash via RS485, hangup the bootloader by requesting a start of the
    // (non-programmed) application, and ask the user to power-cycle the target device.
    // On power-up, the bootloader detects that there is no valid firmware, and enters
    // programming mode with onboard USB programming enabled.
    if (szPortProgram != szPortRequest)
    {
        std::cout << "invalidate firmware ..." << std::endl;
        oAns = oTarget.send(TmclCmd(1, TMCL_BootEraseAll, 0, 0, 0), 10000); // erase flash (may take some time; set 10 seconds timeout)
        if (!oAns.valid())
            return -1;
        std::cout << " ... done." << std::endl;
        std::cout << "shutdown bootloader ..." << std::endl;
        oAns = oTarget.send(TmclCmd(1, TMCL_BootStartAppl, 0, 0, 0)); // boot into application
        if (!oAns.valid())
            return -1;
        std::cout << " ... done." << std::endl;
        std::cout << "please power-cycle the device." << std::endl;
        for (;;)
        {
            oTarget.flush();
            oAns = oTarget.send(TmclCmd(1, TMCL_GetVersion, 1, 0, 0)); // get bootloader version (type = binary)
            if (oAns.valid())
                break;
        }
    }

    // close serial port, give module time to reboot and reopen serial port
    oTarget.close();
    sleep(3);
    if (!oTarget.open(szPortProgram))
        return -1;

    // check if bootloader is ready
    std::cout << "checking for bootloader ..." << std::endl;
    oTarget.flush();
    for (int nRetry = 0; nRetry < 3; nRetry++)
    {
        oTarget.flush();
        oAns = oTarget.send(TmclCmd(1, TMCL_GetVersion, 1, 0, 0)); // get bootloader version (type = binary)
        if (oAns.valid())
            break;
    }
    if (!oAns.valid())
    {
        std::cout << "failed to request bootloader." << std::endl;
        return -1;
    }
    unsigned int nVersion = oAns.value();
    std::cout << "module " << (nVersion >> 16) << ", bootloader version " << ((nVersion >> 8) & 0xff) << "." << outputDec2(nVersion & 0xff) << std::endl;

    oAns = oTarget.send(TmclCmd(1, TMCL_BootGetInfo, 2, 0, 0)); // query total flash size
    if (!oAns.valid())
        return -1;
    int nFlashSize = oAns.value();
    oAns = oTarget.send(TmclCmd(1, TMCL_BootGetInfo, 0, 0, 0)); // query page size
    if (!oAns.valid())
        return -1;
    int nPageSize = oAns.value();
    std::cout << "flash size: " << outputHex8(nFlashSize) << ", " << "page size: " << outputHex8(nPageSize) << std::endl;
    if ((oImage.beg() != 0x4000) || (oImage.end() >= nFlashSize - nPageSize))
    {
        std::cout << "error: image begin " << outputHex8(oImage.beg()) << " not at 0x00004000 or "
            << "image end " << outputHex8(oImage.end()) << " beyond useable flash size " << outputHex8((nFlashSize) - nPageSize) << std::endl;
        return -1;
    }

    std::cout << "erasing flash ..." << std::endl;
    oAns = oTarget.send(TmclCmd(1, TMCL_BootEraseAll, 0, 0, 0), 10000); // erase flash (may take some time; set 10 seconds timeout)
    if (!oAns.valid())
        return -1;
    std::cout << " ... done." << std::endl;

    for (int nAddr = oImage.beg() / nPageSize * nPageSize; nAddr < ((oImage.end() - 1) / nPageSize + 1) * nPageSize; nAddr += nPageSize)
    {
        std::cout << "sending page data ..." << std::endl;
        for (int nIx = 0; nIx < nPageSize / 4; nIx++)
        {
            if ((nAddr + nIx * 4 < oImage.beg()) || (nAddr + nIx * 4 > oImage.end()))
                continue;
            oAns = oTarget.send(TmclCmd(1, TMCL_BootWriteBuffer, nIx, 0, oImage.data(), nAddr + nIx * 4)); // send data
            if (!oAns.valid())
                return -1;
        }
        std::cout << "flashing to physical address " << outputHex8(nAddr) << " ..." << std::endl;
        oAns = oTarget.send(TmclCmd(1, TMCL_BootWritePage, 0, 0, nAddr), 10000); // program flash (may take some time; set 10 seconds timeout)
        if (!oAns.valid())
            return -1;
    }

    std::cout << "query checksum ..." << std::endl;
    oAns = oTarget.send(TmclCmd(1, TMCL_BootGetChecksum, 0, 0, oImage.end() - 1)); // query checksum (data is length-1, not length!)
    if (!oAns.valid())
        return -1;
    if (oAns.value() == nChecksum)
        std::cout << " ... match." << std::endl;
    else
    {
        std::cout << " ... mismatch: got " << outputHex8(oAns.value()) << ", expected " << outputHex8(nChecksum) << std::endl;
        return -1;
    }

    std::cout << "sending length/checksum page data ..." << std::endl;
    oAns = oTarget.send(TmclCmd(1, TMCL_BootWriteBuffer, nPageSize / 4 - 2, 0, oImage.end() - oImage.beg())); // send length
    if (!oAns.valid())
        return -1;
    oAns = oTarget.send(TmclCmd(1, TMCL_BootWriteBuffer, nPageSize / 4 - 1, 0, nChecksum)); // send checksum
    if (!oAns.valid())
        return -1;

    std::cout << "flashing to physical address " << outputHex8((nFlashSize - nPageSize)) << " ..." << std::endl;
    oAns = oTarget.send(TmclCmd(1, TMCL_BootWritePage, 0, 0, nFlashSize - nPageSize), 10000); // program flash (may take some time; set 10 seconds timeout)
    if (!oAns.valid())
        return -1;
    std::cout << " ... done." << std::endl;

    std::cout << "boot into application ..." << std::endl;
    oAns = oTarget.send(TmclCmd(1, TMCL_BootStartAppl, 0, 0, 0)); // boot into application
    if (!oAns.valid())
        return -1;
    std::cout << " ... done." << std::endl;

    oTarget.close();
    return 0;
}
