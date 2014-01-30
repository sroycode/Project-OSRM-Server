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

#include "EdgeBasedGraphFactory.h"

EdgeBasedGraphFactory::EdgeBasedGraphFactory(
    int number_of_nodes,
    std::vector<ImportEdge> & input_edge_list,
    std::vector<NodeID> & barrier_node_list,
    std::vector<NodeID> & traffic_light_node_list,
    std::vector<TurnRestriction> & input_restrictions_list,
    std::vector<NodeInfo> & m_node_info_list,
    SpeedProfileProperties speed_profile
) : speed_profile(speed_profile),
    m_turn_restrictions_count(0),
    m_node_info_list(m_node_info_list)
{
	BOOST_FOREACH(const TurnRestriction & restriction, input_restrictions_list) {
        std::pair<NodeID, NodeID> restriction_source =
            std::make_pair(restriction.fromNode, restriction.viaNode);
        unsigned index;
        RestrictionMap::iterator restriction_iter = m_restriction_map.find(restriction_source);
        if(restriction_iter == m_restriction_map.end()) {
            index = m_restriction_bucket_list.size();
            m_restriction_bucket_list.resize(index+1);
            m_restriction_map.emplace(restriction_source, index);
        } else {
            index = restriction_iter->second;
            //Map already contains an is_only_*-restriction
            if(m_restriction_bucket_list.at(index).begin()->second) {
                continue;
            } else if(restriction.flags.isOnly) {
                //We are going to insert an is_only_*-restriction. There can be only one.
                m_turn_restrictions_count -= m_restriction_bucket_list.at(index).size();
                m_restriction_bucket_list.at(index).clear();
            }
        }
        ++m_turn_restrictions_count;
        m_restriction_bucket_list.at(index).push_back(
            std::make_pair( restriction.toNode, restriction.flags.isOnly)
        );
    }

	m_barrier_nodes.insert(
        barrier_node_list.begin(),
        barrier_node_list.end()
    );

    m_traffic_lights.insert(
        traffic_light_node_list.begin(),
        traffic_light_node_list.end()
    );

    DeallocatingVector< NodeBasedEdge > edges_list;
    NodeBasedEdge edge;
    BOOST_FOREACH(const ImportEdge & import_edge, input_edge_list) {
        if(!import_edge.isForward()) {
            edge.source = import_edge.target();
            edge.target = import_edge.source();
            edge.data.backward = import_edge.isForward();
            edge.data.forward = import_edge.isBackward();
        } else {
            edge.source = import_edge.source();
            edge.target = import_edge.target();
            edge.data.forward = import_edge.isForward();
            edge.data.backward = import_edge.isBackward();
        }
        if(edge.source == edge.target) {
        	continue;
        }
        edge.data.distance = (std::max)((int)import_edge.weight(), 1 );
        assert( edge.data.distance > 0 );
        edge.data.shortcut = false;
        edge.data.roundabout = import_edge.isRoundabout();
        edge.data.ignoreInGrid = import_edge.ignoreInGrid();
        edge.data.nameID = import_edge.name();
        edge.data.type = import_edge.type();
        edge.data.isAccessRestricted = import_edge.isAccessRestricted();
        edge.data.edgeBasedNodeID = edges_list.size();
        edge.data.contraFlow = import_edge.isContraFlow();
        edges_list.push_back( edge );
        if( edge.data.backward ) {
            std::swap( edge.source, edge.target );
            edge.data.forward = import_edge.isBackward();
            edge.data.backward = import_edge.isForward();
            edge.data.edgeBasedNodeID = edges_list.size();
            edges_list.push_back( edge );
        }
    }
    std::vector<ImportEdge>().swap(input_edge_list);
    std::sort( edges_list.begin(), edges_list.end() );
    m_node_based_graph = boost::make_shared<NodeBasedDynamicGraph>(
        number_of_nodes, edges_list
    );
}

void EdgeBasedGraphFactory::GetEdgeBasedEdges(
    DeallocatingVector< EdgeBasedEdge >& output_edge_list
) {
    BOOST_ASSERT_MSG(
        0 == output_edge_list.size(),
        "Vector is not empty"
    );
    m_edge_based_edge_list.swap(output_edge_list);
}

void EdgeBasedGraphFactory::GetEdgeBasedNodes( std::vector<EdgeBasedNode> & nodes) {
#ifndef NDEBUG
    BOOST_FOREACH(const EdgeBasedNode & node, m_edge_based_node_list){
        assert(node.lat1 != INT_MAX); assert(node.lon1 != INT_MAX);
        assert(node.lat2 != INT_MAX); assert(node.lon2 != INT_MAX);
    }
#endif
    nodes.swap(m_edge_based_node_list);
}

NodeID EdgeBasedGraphFactory::CheckForEmanatingIsOnlyTurn(
    const NodeID u,
    const NodeID v
) const {
    const std::pair < NodeID, NodeID > restriction_source = std::make_pair(u, v);
    RestrictionMap::const_iterator restriction_iter = m_restriction_map.find(restriction_source);
    if (restriction_iter != m_restriction_map.end()) {
        const unsigned index = restriction_iter->second;
        BOOST_FOREACH(
            const RestrictionSource & restriction_target,
            m_restriction_bucket_list.at(index)
        ) {
            if(restriction_target.second) {
                return restriction_target.first;
            }
        }
    }
    return UINT_MAX;
}

bool EdgeBasedGraphFactory::CheckIfTurnIsRestricted(
    const NodeID u,
    const NodeID v,
    const NodeID w
) const {
    //only add an edge if turn is not a U-turn except it is the end of dead-end street.
    const std::pair < NodeID, NodeID > restriction_source = std::make_pair(u, v);
    RestrictionMap::const_iterator restriction_iter = m_restriction_map.find(restriction_source);
    if (restriction_iter != m_restriction_map.end()) {
        const unsigned index = restriction_iter->second;
        BOOST_FOREACH(
            const RestrictionTarget & restriction_target,
            m_restriction_bucket_list.at(index)
        ) {
            if(w == restriction_target.first) {
                return true;
            }
        }
    }
    return false;
}

void EdgeBasedGraphFactory::InsertEdgeBasedNode(
        EdgeIterator e1,
        NodeIterator u,
        NodeIterator v,
        bool belongsToTinyComponent) {
    EdgeData & data = m_node_based_graph->GetEdgeData(e1);
    EdgeBasedNode currentNode;
    currentNode.nameID = data.nameID;
    currentNode.lat1 = m_node_info_list[u].lat;
    currentNode.lon1 = m_node_info_list[u].lon;
    currentNode.lat2 = m_node_info_list[v].lat;
    currentNode.lon2 = m_node_info_list[v].lon;
    currentNode.belongsToTinyComponent = belongsToTinyComponent;
    currentNode.id = data.edgeBasedNodeID;
    currentNode.ignoreInGrid = data.ignoreInGrid;
    currentNode.weight = data.distance;
    m_edge_based_node_list.push_back(currentNode);
}

unsigned EdgeBasedGraphFactory::GetNumberOfNodes() const {
    return m_node_based_graph->GetNumberOfEdges();
}
