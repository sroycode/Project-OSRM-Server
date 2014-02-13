/*

Copyright (c) 2013, Project OSRM, Dennis Luxen, others
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.
Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef POLYLINECOMPRESSOR_H_
#define POLYLINECOMPRESSOR_H_

#include "../DataStructures/SegmentInformation.h"
#include "../Util/StringUtil.h"

#include <string>
#include <vector>

class PolylineCompressor {
private:
	void encodeVectorSignedNumber(
        std::vector<int> & numbers,
        std::string & output
    ) const;

	void encodeNumber(int number_to_encode, std::string & output) const;

public:
    void printEncodedString(
        const std::vector<SegmentInformation> & polyline,
        std::string & output
    ) const;

    void printEncodedString(
        const std::vector<FixedPointCoordinate>& polyline,
        std::string &output
    ) const;

    void printUnencodedString(
        const std::vector<FixedPointCoordinate> & polyline,
        std::string & output
    ) const;

    void printUnencodedString(
        const std::vector<SegmentInformation> & polyline,
        std::string & output
    ) const;

};

#endif /* POLYLINECOMPRESSOR_H_ */
