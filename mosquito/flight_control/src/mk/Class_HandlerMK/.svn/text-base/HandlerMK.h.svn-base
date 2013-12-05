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
#ifndef HANDLERMK_H
#define HANDLERMK_H


#include "Kopter.h"
#include "MK_Datatypes.h"
#include "stdio.h"
#include <stdlib.h>
#include <stdio.h>
using namespace std;

struct s_Hardware
{
    int ID;
    int VERSION_MAJOR;
    int VERSION_MINOR;
    int VERSION_PATCH;
    int VERSION_SERIAL_MAJOR;
    int VERSION_SERIAL_MINOR;
    string Hardware;
    string Version;
    string VersionShort;
};


class HandlerMK
{
    public:
        HandlerMK();

        static string add_CRC(string TXString);
        static bool Check_CRC(char *t_InData, int Length);

        static int Decode_64(char *t_InData, int Length, unsigned char *t_OutData);
        static string Encode64(char Data[150],unsigned int Length);
        static string make_Frame(char t_CMD, int t_Adress, char t_Data[150], unsigned int t_Length);

        static string get_SelectFC();
        static string get_SelectNC();
        static string get_SelectMK3MAG();
        static int Data2Int(unsigned char Data[150] , int Start, bool is_signed = true);

};

#endif // HANDLERMK_H
