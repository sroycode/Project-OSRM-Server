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

#ifndef DYNAMICGRAPH_H_INCLUDED
#define DYNAMICGRAPH_H_INCLUDED

#include "../DataStructures/DeallocatingVector.h"

#include <boost/assert.hpp>
#include <boost/integer.hpp>

#include <algorithm>
#include <limits>
#include <vector>

template< typename EdgeDataT>
class DynamicGraph {
    public:
        typedef EdgeDataT EdgeData;
        typedef uint32_t NodeIterator;
        typedef uint32_t EdgeIterator;

        class InputEdge {
            public:
                NodeIterator source;
                NodeIterator target;
                EdgeDataT data;
                bool operator<( const InputEdge& right ) const {
                    if ( source != right.source )
                        return source < right.source;
                    return target < right.target;
                }
        };

        //Constructs an empty graph with a given number of nodes.
        DynamicGraph( int32_t nodes ) : m_numNodes(nodes), m_numEdges(0) {
            m_nodes.reserve( m_numNodes );
            m_nodes.resize( m_numNodes );

            m_edges.reserve( m_numNodes * 1.1 );
            m_edges.resize( m_numNodes );
        }

        template<class ContainerT>
        DynamicGraph( const int32_t nodes, const ContainerT &graph ) {
            m_numNodes = nodes;
            m_numEdges = ( EdgeIterator ) graph.size();
            m_nodes.reserve( m_numNodes +1);
            m_nodes.resize( m_numNodes +1);
            EdgeIterator edge = 0;
            EdgeIterator position = 0;
            for ( NodeIterator node = 0; node < m_numNodes; ++node ) {
                EdgeIterator lastEdge = edge;
                while ( edge < m_numEdges && graph[edge].source == node ) {
                    ++edge;
                }
                m_nodes[node].firstEdge = position;
                m_nodes[node].edges = edge - lastEdge;
                position += m_nodes[node].edges;
            }
            m_nodes.back().firstEdge = position;
            m_edges.reserve( position * 1.1 );
            m_edges.resize( position );
            edge = 0;
            for ( NodeIterator node = 0; node < m_numNodes; ++node ) {
                for ( EdgeIterator i = m_nodes[node].firstEdge, e = m_nodes[node].firstEdge + m_nodes[node].edges; i != e; ++i ) {
                    m_edges[i].target = graph[edge].target;
                    m_edges[i].data = graph[edge].data;
                    BOOST_ASSERT_MSG(
                        graph[edge].data.distance > 0,
                        "edge distance invalid"
                    );
                    ++edge;
                }
            }
        }

        ~DynamicGraph(){ }

        uint32_t GetNumberOfNodes() const {
            return m_numNodes;
        }

        uint32_t GetNumberOfEdges() const {
            return m_numEdges;
        }

        uint32_t GetOutDegree( const NodeIterator n ) const {
            return m_nodes[n].edges;
        }

        NodeIterator GetTarget( const EdgeIterator e ) const {
            return NodeIterator( m_edges[e].target );
        }

        EdgeDataT &GetEdgeData( const EdgeIterator e ) {
            return m_edges[e].data;
        }

        const EdgeDataT &GetEdgeData( const EdgeIterator e ) const {
            return m_edges[e].data;
        }

        EdgeIterator BeginEdges( const NodeIterator n ) const {
            return EdgeIterator( m_nodes[n].firstEdge );
        }

        EdgeIterator EndEdges( const NodeIterator n ) const {
            return EdgeIterator( m_nodes[n].firstEdge + m_nodes[n].edges );
        }

        //adds an edge. Invalidates edge iterators for the source node
        EdgeIterator InsertEdge( const NodeIterator from, const NodeIterator to, const EdgeDataT &data ) {
            Node &node = m_nodes[from];
            EdgeIterator newFirstEdge = node.edges + node.firstEdge;
            if ( newFirstEdge >= m_edges.size() || !isDummy( newFirstEdge ) ) {
                if ( node.firstEdge != 0 && isDummy( node.firstEdge - 1 ) ) {
                    node.firstEdge--;
                    m_edges[node.firstEdge] = m_edges[node.firstEdge + node.edges];
                } else {
                    EdgeIterator newFirstEdge = ( EdgeIterator ) m_edges.size();
                    uint32_t newSize = node.edges * 1.1 + 2;
                    EdgeIterator requiredCapacity = newSize + m_edges.size();
                    EdgeIterator oldCapacity = m_edges.capacity();
                    if ( requiredCapacity >= oldCapacity ) {
                        m_edges.reserve( requiredCapacity * 1.1 );
                    }
                    m_edges.resize( m_edges.size() + newSize );
                    for ( EdgeIterator i = 0; i < node.edges; ++i ) {
                        m_edges[newFirstEdge + i ] = m_edges[node.firstEdge + i];
                        makeDummy( node.firstEdge + i );
                    }
                    for ( EdgeIterator i = node.edges + 1; i < newSize; ++i )
                        makeDummy( newFirstEdge + i );
                    node.firstEdge = newFirstEdge;
                }
            }
            Edge &edge = m_edges[node.firstEdge + node.edges];
            edge.target = to;
            edge.data = data;
            ++m_numEdges;
            ++node.edges;
            return EdgeIterator( node.firstEdge + node.edges );
        }

        //removes an edge. Invalidates edge iterators for the source node
        void DeleteEdge( const NodeIterator source, const EdgeIterator e ) {
            Node &node = m_nodes[source];
            --m_numEdges;
            --node.edges;
            const uint32_t last = node.firstEdge + node.edges;
            //swap with last edge
            m_edges[e] = m_edges[last];
            makeDummy( last );
        }

        //removes all edges (source,target)
        int32_t DeleteEdgesTo( const NodeIterator source, const NodeIterator target ) {
            int32_t deleted = 0;
            for ( EdgeIterator i = BeginEdges( source ), iend = EndEdges( source ); i < iend - deleted; ++i ) {
                if ( m_edges[i].target == target ) {
                    do {
                        deleted++;
                        m_edges[i] = m_edges[iend - deleted];
                        makeDummy( iend - deleted );
                    } while ( i < iend - deleted && m_edges[i].target == target );
                }
            }

            #pragma omp atomic
            m_numEdges -= deleted;
            m_nodes[source].edges -= deleted;

            return deleted;
        }

        //searches for a specific edge
        EdgeIterator FindEdge( const NodeIterator from, const NodeIterator to ) const {
            for ( EdgeIterator i = BeginEdges( from ), iend = EndEdges( from ); i != iend; ++i ) {
                if ( to == m_edges[i].target ) {
                    return i;
                }
            }
            return EndEdges( from );
        }

    protected:

        bool isDummy( const EdgeIterator edge ) const {
            return m_edges[edge].target == (std::numeric_limits< NodeIterator >::max)();
        }

        void makeDummy( const EdgeIterator edge ) {
            m_edges[edge].target = (std::numeric_limits< NodeIterator >::max)();
        }

        struct Node {
            //index of the first edge
            EdgeIterator firstEdge;
            //amount of edges
            uint32_t edges;
        };

        struct Edge {
            NodeIterator target;
            EdgeDataT data;
        };

        NodeIterator m_numNodes;
        EdgeIterator m_numEdges;

        std::vector< Node > m_nodes;
        DeallocatingVector< Edge > m_edges;
};

#endif // DYNAMICGRAPH_H_INCLUDED
