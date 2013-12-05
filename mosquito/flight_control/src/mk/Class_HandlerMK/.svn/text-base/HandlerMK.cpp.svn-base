/***************************************************************************
 *   Copyright (C) 2009 by Manuel Schrape                                  *
 *   manuel.schrape@gmx.de                                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License.        *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#include <string>
#include <stdlib.h>
#include <sstream>


#include "HandlerMK.h"
using namespace std;

HandlerMK::HandlerMK()
{
}



string HandlerMK::add_CRC(string TXString)
{
    unsigned int tmpCRC = 0;

    const char *TXBuff;
    char CRC[2];

    TXBuff = TXString.data();

    for(unsigned int i = 0; i < TXString.length(); i++)
    {
        tmpCRC += TXBuff[i];
    }

    tmpCRC %= 4096;

    CRC[0] = '=' + tmpCRC / 64;
    CRC[1] = '=' + tmpCRC % 64;
    CRC[2] = '\0';

    string Return = TXString + string(CRC);

    return Return;
}

bool HandlerMK::Check_CRC(char *t_InData, int Length)
{
    int CRC = 0;

    if (t_InData[1] == 127)
        t_InData[1] = 0;

    for(int i=0; i < Length-2; i++)
        CRC+=t_InData[i];

    CRC = CRC % 4096;

    if(t_InData[Length - 2] != ('=' + (CRC / 64)))
        return false;

    if(t_InData[Length - 1] != ('=' + CRC % 64))
        return false;

    return true;
}

int HandlerMK::Decode_64(char *t_InData, int Length, unsigned char *t_OutData)
{
    unsigned char a,b,c,d;
    unsigned char ptr = 0;
    unsigned char x,y,z;

    int Offset = 3;//how many elements are in the header

    if (t_InData[Offset] == 0)
    {
        return 0;
    }

    while(Length != 0)
    {
        a = t_InData[Offset++] - '=';
        b = t_InData[Offset++] - '=';
        c = t_InData[Offset++] - '=';
        d = t_InData[Offset++] - '=';

//        if(ptrIn > max - 2) break; // nicht mehr Daten verarbeiten, als empfangen wurden

        x = (a << 2) | (b >> 4);
        y = ((b & 0x0f) << 4) | (c >> 2);
        z = ((c & 0x03) << 6) | d;

        if(Length--) t_OutData[ptr++] = x; else break;
        if(Length--) t_OutData[ptr++] = y; else break;
        if(Length--) t_OutData[ptr++] = z; else break;
    }

    return ptr;
}


// Base64 Encoder
string HandlerMK::Encode64(char Data[150],unsigned int Length)
{
    unsigned int pt = 0;
    unsigned char a,b,c;
    unsigned char ptr = 0;

    char TX_Buff[150];

    while(Length > 0)
    {
        if(Length) { a = Data[ptr++]; Length--;} else a = 0;
        if(Length) { b = Data[ptr++]; Length--;} else b = 0;
        if(Length) { c = Data[ptr++]; Length--;} else c = 0;

        TX_Buff[pt++] = '=' + (a >> 2);
        TX_Buff[pt++] = '=' + (((a & 0x03) << 4) | ((b & 0xf0) >> 4));
        TX_Buff[pt++] = '=' + (((b & 0x0f) << 2) | ((c & 0xc0) >> 6));
        TX_Buff[pt++] = '=' + ( c & 0x3f);
    }
    TX_Buff[pt] = 0;

    return string(TX_Buff);
}

string HandlerMK::make_Frame(char t_CMD, int t_Adress, char t_Data[150], unsigned int t_Length)
{
    string tx_Data = Encode64(t_Data, t_Length);
    stringstream st;
    st << "#a" << t_CMD << tx_Data ;


    return  add_CRC(st.str())+"\r";
}

string HandlerMK::get_SelectNC()
{
    char t_Data[6];

    t_Data[0] = 0x1B;
    t_Data[1] = 0x1B;
    t_Data[2] = 0x55;
    t_Data[3] = 0xAA;
    t_Data[4] = 0x00;
    t_Data[5] = '\r';

    string tx_Data = string(t_Data);

    return tx_Data;
}

string HandlerMK::get_SelectFC()
{
    char t_Data[1];

    t_Data[0] = 0x00;

    string tx_Data = Encode64(t_Data, 1);
    string temp;
    temp.append("#cu");
    temp.append(tx_Data);

    tx_Data = add_CRC(temp) + "\r";

    return tx_Data;
}

string HandlerMK::get_SelectMK3MAG()
{
    char t_Data[1];

    t_Data[0] = 1;

    string tx_Data = Encode64(t_Data, 1);
    string temp;
    temp.append("#a3u");
    temp.append(tx_Data);

    tx_Data = add_CRC(temp) + "\r";

    return tx_Data;
}

// Datensatz nach 16bit Integer
int HandlerMK::Data2Int(unsigned char Data[150] , int Start, bool is_signed)
{
    int Out = (Data[Start+1]<<8) | (Data[Start+0]);

    if ((Out > 32767) && (is_signed))
      Out = Out - 65536;

    return Out;

}

