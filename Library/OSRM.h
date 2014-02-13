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

#ifndef OSRM_H
#define OSRM_H

#include "OSRM.h"

#include "../Plugins/BasePlugin.h"
#include "../Plugins/HelloWorldPlugin.h"
#include "../Plugins/LocatePlugin.h"
#include "../Plugins/NearestPlugin.h"
#include "../Plugins/TimestampPlugin.h"
#include "../Plugins/ViaRoutePlugin.h"
#include "../Server/DataStructures/BaseDataFacade.h"
#include "../Server/DataStructures/InternalDataFacade.h"
#include "../Server/DataStructures/SharedBarriers.h"
#include "../Server/DataStructures/SharedDataFacade.h"
#include "../Server/DataStructures/RouteParameters.h"
#include "../Util/InputFileUtil.h"
#include "../Util/OSRMException.h"
#include "../Util/SimpleLogger.h"

#include <boost/assert.hpp>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>
#include <boost/interprocess/shared_memory_object.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <vector>

class OSRM : boost::noncopyable {
private:
    typedef boost::unordered_map<std::string, BasePlugin *> PluginMap;
public:
    OSRM(
        const ServerPaths & paths,
        const bool use_shared_memory = false
    );
    ~OSRM();
    void RunQuery(RouteParameters & route_parameters, http::Reply & reply);

private:
    void RegisterPlugin(BasePlugin * plugin);
    PluginMap plugin_map;
    bool use_shared_memory;
    SharedBarriers barrier;
    //base class pointer to the objects
    BaseDataFacade<QueryEdge::EdgeData> * query_data_facade;
};

#endif //OSRM_H
