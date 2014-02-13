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

#ifndef BASEPLUGIN_H_
#define BASEPLUGIN_H_

#include "../DataStructures/Coordinate.h"
#include "../Server/DataStructures/RouteParameters.h"
#include "../Server/Http/Reply.h"

#include <string>
#include <vector>

class BasePlugin {
public:
	BasePlugin() { }
	//Maybe someone can explain the pure virtual destructor thing to me (dennis)
	virtual ~BasePlugin() { }
	virtual const std::string & GetDescriptor() const = 0;
	virtual void HandleRequest(const RouteParameters & routeParameters, http::Reply& reply) = 0;

	inline bool checkCoord(const FixedPointCoordinate & c) {
        if(
            c.lat >   90*COORDINATE_PRECISION ||
            c.lat <  -90*COORDINATE_PRECISION ||
            c.lon >  180*COORDINATE_PRECISION ||
            c.lon < -180*COORDINATE_PRECISION
        ) {
            return false;
        }
        return true;
    }
};

#endif /* BASEPLUGIN_H_ */
