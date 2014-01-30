/*
    open source routing machine
    Copyright (C) Dennis Luxen, others 2010

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU AFFERO General Public License as published by
the Free Software Foundation; either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Affero General Public License
along with this program; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
or see http://www.gnu.org/licenses/agpl.txt.
 */

#ifndef FASTXORHASH_H_
#define FASTXORHASH_H_

#include <algorithm>
#include <vector>

/*
    This is an implementation of Tabulation hashing, which has suprising properties like universality.
    The space requirement is 2*2^16 = 256 kb of memory, which fits into L2 cache.
    Evaluation boils down to 10 or less assembly instruction on any recent X86 CPU:

    1: movq    table2(%rip), %rdx
    2: movl    %edi, %eax
    3: movzwl  %di, %edi
    4: shrl    $16, %eax
    5: movzwl  %ax, %eax
    6: movzbl  (%rdx,%rax), %eax
    7: movq    table1(%rip), %rdx
    8: xorb    (%rdx,%rdi), %al
    9: movzbl  %al, %eax
    10: ret

*/
class XORFastHash { //65k entries
    std::vector<unsigned short> table1;
    std::vector<unsigned short> table2;
public:
    XORFastHash() {
        table1.resize(2 << 16);
        table2.resize(2 << 16);
        for(unsigned i = 0; i < (2 << 16); ++i) {
            table1[i] = i; table2[i] = i;
        }
        std::random_shuffle(table1.begin(), table1.end());
        std::random_shuffle(table2.begin(), table2.end());
    }

    inline unsigned short operator()(const unsigned originalValue) const {
        unsigned short lsb = ((originalValue) & 0xffff);
        unsigned short msb = (((originalValue) >> 16) & 0xffff);
        return table1[lsb] ^ table2[msb];
    }
};

class XORMiniHash { //256 entries
    std::vector<unsigned char> table1;
    std::vector<unsigned char> table2;
    std::vector<unsigned char> table3;
    std::vector<unsigned char> table4;

public:
    XORMiniHash() {
        table1.resize(1 << 8);
        table2.resize(1 << 8);
        table3.resize(1 << 8);
        table4.resize(1 << 8);
        for(unsigned i = 0; i < (1 << 8); ++i) {
            table1[i] = i; table2[i] = i;
            table3[i] = i; table4[i] = i;
        }
        std::random_shuffle(table1.begin(), table1.end());
        std::random_shuffle(table2.begin(), table2.end());
        std::random_shuffle(table3.begin(), table3.end());
        std::random_shuffle(table4.begin(), table4.end());
    }
    unsigned char operator()(const unsigned originalValue) const {
        unsigned char byte1 = ((originalValue) & 0xff);
        unsigned char byte2 = ((originalValue >> 8) & 0xff);
        unsigned char byte3 = ((originalValue >> 16) & 0xff);
        unsigned char byte4 = ((originalValue >> 24) & 0xff);
        return table1[byte1] ^ table2[byte2] ^ table3[byte3] ^ table4[byte4];
    }
};

#endif /* FASTXORHASH_H_ */
