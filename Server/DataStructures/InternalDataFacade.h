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

#ifndef INTERNAL_DATA_FACADE
#define INTERNAL_DATA_FACADE

//implements all data storage when shared memory is _NOT_ used

#include "BaseDataFacade.h"

#include "../../DataStructures/Coordinate.h"
#include "../../DataStructures/QueryNode.h"
#include "../../DataStructures/QueryEdge.h"
#include "../../DataStructures/SharedMemoryVectorWrapper.h"
#include "../../DataStructures/StaticGraph.h"
#include "../../DataStructures/StaticRTree.h"
#include "../../Util/BoostFileSystemFix.h"
#include "../../Util/GraphLoader.h"
#include "../../Util/IniFile.h"
#include "../../Util/ProgramOptions.h"
#include "../../Util/SimpleLogger.h"

template<class EdgeDataT>
class InternalDataFacade : public BaseDataFacade<EdgeDataT> {

private:
    typedef BaseDataFacade<EdgeDataT>             super;
    typedef StaticGraph<typename super::EdgeData> QueryGraph;
    typedef typename QueryGraph::InputEdge        InputEdge;
    typedef typename super::RTreeLeaf             RTreeLeaf;

    InternalDataFacade() { }

    unsigned                                 m_check_sum;
    unsigned                                 m_number_of_nodes;
    QueryGraph                             * m_query_graph;
    std::string                              m_timestamp;

    ShM<FixedPointCoordinate, false>::vector m_coordinate_list;
    ShM<NodeID, false>::vector               m_via_node_list;
    ShM<unsigned, false>::vector             m_name_ID_list;
    ShM<TurnInstruction, false>::vector      m_turn_instruction_list;
    ShM<char, false>::vector                 m_names_char_list;
    ShM<unsigned, false>::vector             m_name_begin_indices;

    StaticRTree<RTreeLeaf, false>          * m_static_rtree;


    void LoadTimestamp(const boost::filesystem::path & timestamp_path) {
        if( boost::filesystem::exists(timestamp_path) ) {
            SimpleLogger().Write() << "Loading Timestamp";
            boost::filesystem::ifstream timestampInStream( timestamp_path );
            if(!timestampInStream) {
                SimpleLogger().Write(logWARNING) << timestamp_path << " not found";
            }
            getline(timestampInStream, m_timestamp);
            timestampInStream.close();
        }
        if(m_timestamp.empty()) {
            m_timestamp = "n/a";
        }
        if(25 < m_timestamp.length()) {
            m_timestamp.resize(25);
        }
    }

    void LoadGraph(const boost::filesystem::path & hsgr_path) {
        typename ShM<typename QueryGraph::_StrNode, false>::vector node_list;
        typename ShM<typename QueryGraph::_StrEdge, false>::vector edge_list;

        SimpleLogger().Write() << "loading graph from " << hsgr_path.string();

        m_number_of_nodes = readHSGRFromStream(
            hsgr_path,
            node_list,
            edge_list,
            &m_check_sum
        );

        BOOST_ASSERT_MSG(0 != node_list.size(), "node list empty");
        BOOST_ASSERT_MSG(0 != edge_list.size(), "edge list empty");
        SimpleLogger().Write() << "loaded " << node_list.size() << " nodes and " << edge_list.size() << " edges";
        m_query_graph = new QueryGraph(node_list, edge_list);

        BOOST_ASSERT_MSG(0 == node_list.size(), "node list not flushed");
        BOOST_ASSERT_MSG(0 == edge_list.size(), "edge list not flushed");
        SimpleLogger().Write() << "Data checksum is " << m_check_sum;
    }

    void LoadNodeAndEdgeInformation(
        const boost::filesystem::path nodes_file,
        const boost::filesystem::path edges_file
    ) {
        boost::filesystem::ifstream nodes_input_stream(
            nodes_file,
            std::ios::binary
        );

        SimpleLogger().Write(logDEBUG) << "Loading node data";
        NodeInfo current_node;
        unsigned number_of_coordinates = 0;
        nodes_input_stream.read(
            (char *)&number_of_coordinates,
            sizeof(unsigned)
        );
        m_coordinate_list.resize(number_of_coordinates);
        for(unsigned i = 0; i < number_of_coordinates; ++i) {
            nodes_input_stream.read((char *)&current_node, sizeof(NodeInfo));
            m_coordinate_list[i] = FixedPointCoordinate(
                    current_node.lat,
                    current_node.lon
            );
        }
        std::vector<FixedPointCoordinate>(m_coordinate_list).swap(m_coordinate_list);
        nodes_input_stream.close();

        SimpleLogger().Write(logDEBUG) << "Loading edge data";
        boost::filesystem::ifstream edges_input_stream(
            edges_file,
            std::ios::binary
        );
        unsigned number_of_edges = 0;
        edges_input_stream.read((char*)&number_of_edges, sizeof(unsigned));
        m_via_node_list.resize(number_of_edges);
        m_name_ID_list.resize(number_of_edges);
        m_turn_instruction_list.resize(number_of_edges);

        OriginalEdgeData current_edge_data;
        for(unsigned i = 0; i < number_of_edges; ++i) {
            edges_input_stream.read(
                (char*)&(current_edge_data),
                sizeof(OriginalEdgeData)
            );
            m_via_node_list[i] = current_edge_data.viaNode;
            m_name_ID_list[i]  = current_edge_data.nameID;
            m_turn_instruction_list[i] = current_edge_data.turnInstruction;
        }
        edges_input_stream.close();
    }

    void LoadRTree(
        const boost::filesystem::path & ram_index_path,
        const boost::filesystem::path & file_index_path
    ) {
        m_static_rtree = new StaticRTree<RTreeLeaf>(
            ram_index_path,
            file_index_path
        );
    }

    void LoadStreetNames(
        const boost::filesystem::path & names_file
    ) {
        boost::filesystem::ifstream name_stream(names_file, std::ios::binary);
        unsigned number_of_names = 0;
        unsigned number_of_chars = 0;
        name_stream.read((char *)&number_of_names, sizeof(unsigned));
        name_stream.read((char *)&number_of_chars, sizeof(unsigned));
        BOOST_ASSERT_MSG(0 != number_of_names, "name file broken");
        BOOST_ASSERT_MSG(0 != number_of_chars, "name file broken");

        m_name_begin_indices.resize(number_of_names);
        name_stream.read(
            (char*)&m_name_begin_indices[0],
            number_of_names*sizeof(unsigned)
        );

        m_names_char_list.resize(number_of_chars+1); //+1 gives sentinel element
        name_stream.read(
            (char *)&m_names_char_list[0],
            number_of_chars*sizeof(char)
        );
        BOOST_ASSERT_MSG(
            0 != m_names_char_list.size(),
            "could not load any names"
        );
        name_stream.close();
    }
public:
    ~InternalDataFacade() {
        delete m_query_graph;
        delete m_static_rtree;
    }

    InternalDataFacade( const ServerPaths & server_paths ) {
        //generate paths of data files
        if( server_paths.find("hsgrdata") == server_paths.end() ) {
            throw OSRMException("no hsgr file given in ini file");
        }
        if( server_paths.find("ramindex") == server_paths.end() ) {
            throw OSRMException("no ram index file given in ini file");
        }
        if( server_paths.find("fileindex") == server_paths.end() ) {
            throw OSRMException("no leaf index file given in ini file");
        }
        if( server_paths.find("nodesdata") == server_paths.end() ) {
            throw OSRMException("no nodes file given in ini file");
        }
        if( server_paths.find("edgesdata") == server_paths.end() ) {
            throw OSRMException("no edges file given in ini file");
        }
        if( server_paths.find("namesdata") == server_paths.end() ) {
            throw OSRMException("no names file given in ini file");
        }

        ServerPaths::const_iterator paths_iterator = server_paths.find("hsgrdata");
        BOOST_ASSERT(server_paths.end() != paths_iterator);
        const boost::filesystem::path & hsgr_path = paths_iterator->second;
        paths_iterator = server_paths.find("timestamp");
        BOOST_ASSERT(server_paths.end() != paths_iterator);
        const boost::filesystem::path & timestamp_path = paths_iterator->second;
        paths_iterator = server_paths.find("ramindex");
        BOOST_ASSERT(server_paths.end() != paths_iterator);
        const boost::filesystem::path & ram_index_path = paths_iterator->second;
        paths_iterator = server_paths.find("fileindex");
        BOOST_ASSERT(server_paths.end() != paths_iterator);
        const boost::filesystem::path & file_index_path = paths_iterator->second;
        paths_iterator = server_paths.find("nodesdata");
        BOOST_ASSERT(server_paths.end() != paths_iterator);
        const boost::filesystem::path & nodes_data_path = paths_iterator->second;
        paths_iterator = server_paths.find("edgesdata");
        BOOST_ASSERT(server_paths.end() != paths_iterator);
        const boost::filesystem::path & edges_data_path = paths_iterator->second;
        paths_iterator = server_paths.find("namesdata");
        BOOST_ASSERT(server_paths.end() != paths_iterator);
        const boost::filesystem::path & names_data_path = paths_iterator->second;

        //load data
        SimpleLogger().Write() << "loading graph data";
        LoadGraph(hsgr_path);
        SimpleLogger().Write() << "loading egde information";
        LoadNodeAndEdgeInformation(nodes_data_path, edges_data_path);
        SimpleLogger().Write() << "loading r-tree";
        LoadRTree(ram_index_path, file_index_path);
        SimpleLogger().Write() << "loading timestamp";
        LoadTimestamp(timestamp_path);
        SimpleLogger().Write() << "loading street names";
        LoadStreetNames(names_data_path);
    }

    //search graph access
    unsigned GetNumberOfNodes() const {
        return m_query_graph->GetNumberOfNodes();
    }

    unsigned GetNumberOfEdges() const {
        return m_query_graph->GetNumberOfEdges();
    }

    unsigned GetOutDegree( const NodeID n ) const {
        return m_query_graph->GetOutDegree(n);
    }

    NodeID GetTarget( const EdgeID e ) const {
        return m_query_graph->GetTarget(e); }

    EdgeDataT &GetEdgeData( const EdgeID e ) {
        return m_query_graph->GetEdgeData(e);
    }

    const EdgeDataT &GetEdgeData( const EdgeID e ) const {
        return m_query_graph->GetEdgeData(e);
    }

    EdgeID BeginEdges( const NodeID n ) const {
        return m_query_graph->BeginEdges(n);
    }

    EdgeID EndEdges( const NodeID n ) const {
        return m_query_graph->EndEdges(n);
    }

    //searches for a specific edge
    EdgeID FindEdge( const NodeID from, const NodeID to ) const {
        return m_query_graph->FindEdge(from, to);
    }

    EdgeID FindEdgeInEitherDirection(
        const NodeID from,
        const NodeID to
    ) const {
        return m_query_graph->FindEdgeInEitherDirection(from, to);
    }

    EdgeID FindEdgeIndicateIfReverse(
        const NodeID from,
        const NodeID to,
        bool & result
    ) const {
        return m_query_graph->FindEdgeIndicateIfReverse(from, to, result);
    }

    //node and edge information access
    FixedPointCoordinate GetCoordinateOfNode(
        const unsigned id
    ) const {
        const NodeID node = m_via_node_list.at(id);
        return m_coordinate_list.at(node);
    };

    TurnInstruction GetTurnInstructionForEdgeID(
        const unsigned id
    ) const {
        return m_turn_instruction_list.at(id);
    }

    bool LocateClosestEndPointForCoordinate(
        const FixedPointCoordinate& input_coordinate,
        FixedPointCoordinate& result,
        const unsigned zoom_level = 18
    ) const {
        return  m_static_rtree->LocateClosestEndPointForCoordinate(
                    input_coordinate,
                    result,
                    zoom_level
                );
    }

    bool FindPhantomNodeForCoordinate(
        const FixedPointCoordinate & input_coordinate,
        PhantomNode & resulting_phantom_node,
        const unsigned zoom_level
    ) const {
        return  m_static_rtree->FindPhantomNodeForCoordinate(
                    input_coordinate,
                    resulting_phantom_node,
                    zoom_level
                );
    }

    unsigned GetCheckSum() const { return m_check_sum; }

    unsigned GetNameIndexFromEdgeID(const unsigned id) const {
        return m_name_ID_list.at(id);
    };

    void GetName( const unsigned name_id, std::string & result ) const {
        if(UINT_MAX == name_id) {
            result = "";
            return;
        }
        BOOST_ASSERT_MSG(
            name_id < m_name_begin_indices.size(),
            "name id too high"
        );
        unsigned begin_index = m_name_begin_indices[name_id];
        unsigned end_index = m_name_begin_indices[name_id+1];
        BOOST_ASSERT_MSG(
            begin_index < m_names_char_list.size(),
            "begin index of name too high"
        );
        BOOST_ASSERT_MSG(
            end_index < m_names_char_list.size(),
            "end index of name too high"
        );

        BOOST_ASSERT_MSG(begin_index <= end_index, "string ends before begin");
        result.clear();
        result.resize(end_index - begin_index);
        std::copy(
            m_names_char_list.begin() + begin_index,
            m_names_char_list.begin() + end_index,
            result.begin()
        );
    }

    std::string GetTimestamp() const {
        return m_timestamp;
    }
};

#endif  // INTERNAL_DATA_FACADE
