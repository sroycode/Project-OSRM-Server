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

#ifndef BASIC_DATASTRUCTURES_H
#define BASIC_DATASTRUCTURES_H

#include "../Util/StringUtil.h"

#include <boost/asio.hpp>
#include <boost/foreach.hpp>

#include <string>
#include <sstream>
#include <vector>

namespace http {

const std::string okString 					= "HTTP/1.0 200 OK\r\n";
const std::string badRequestString 			= "HTTP/1.0 400 Bad Request\r\n";
const std::string internalServerErrorString = "HTTP/1.0 500 Internal Server Error\r\n";

const char okHTML[] 				 = "";
const char badRequestHTML[] 		 = "<html><head><title>Bad Request</title></head><body><h1>400 Bad Request</h1></body></html>";
const char internalServerErrorHTML[] = "<html><head><title>Internal Server Error</title></head><body><h1>500 Internal Server Error</h1></body></html>";
const char seperators[]  			 = { ':', ' ' };
const char crlf[]		             = { '\r', '\n' };

struct Header {
  std::string name;
  std::string value;
  void Clear() {
      name.clear();
      value.clear();
  }
};

enum CompressionType {
    noCompression,
    gzipRFC1952,
    deflateRFC1951
};

struct Request {
	std::string uri;
	std::string referrer;
	std::string agent;
	boost::asio::ip::address endpoint;
};

struct Reply {
    Reply() : status(ok) { content.reserve(2 << 20); }
	enum status_type {
		ok 					= 200,
		badRequest 		    = 400,
		internalServerError = 500
	} status;

	std::vector<Header> headers;
    std::vector<boost::asio::const_buffer> toBuffers();
    std::vector<boost::asio::const_buffer> HeaderstoBuffers();
	std::string content;
	static Reply stockReply(status_type status);
	void setSize(const unsigned size) {
		BOOST_FOREACH ( Header& h,  headers) {
			if("Content-Length" == h.name) {
				std::string sizeString;
				intToString(size,h.value);
			}
		}
	}
};

inline boost::asio::const_buffer ToBuffer(Reply::status_type status) {
	switch (status) {
	case Reply::ok:
		return boost::asio::buffer(okString);
	case Reply::internalServerError:
		return boost::asio::buffer(internalServerErrorString);
	default:
		return boost::asio::buffer(badRequestString);
	}
}

inline std::string ToString(Reply::status_type status) {
	switch (status) {
	case Reply::ok:
		return okHTML;
	case Reply::badRequest:
		return badRequestHTML;
	default:
		return internalServerErrorHTML;
	}
}

inline std::vector<boost::asio::const_buffer> Reply::toBuffers(){
	std::vector<boost::asio::const_buffer> buffers;
	buffers.push_back(ToBuffer(status));
	for (std::size_t i = 0; i < headers.size(); ++i) {
		Header& h = headers[i];
		buffers.push_back(boost::asio::buffer(h.name));
		buffers.push_back(boost::asio::buffer(seperators));
		buffers.push_back(boost::asio::buffer(h.value));
		buffers.push_back(boost::asio::buffer(crlf));
	}
	buffers.push_back(boost::asio::buffer(crlf));
	buffers.push_back(boost::asio::buffer(content));
	return buffers;
}

inline std::vector<boost::asio::const_buffer> Reply::HeaderstoBuffers(){
    std::vector<boost::asio::const_buffer> buffers;
    buffers.push_back(ToBuffer(status));
    for (std::size_t i = 0; i < headers.size(); ++i) {
        Header& h = headers[i];
        buffers.push_back(boost::asio::buffer(h.name));
        buffers.push_back(boost::asio::buffer(seperators));
        buffers.push_back(boost::asio::buffer(h.value));
        buffers.push_back(boost::asio::buffer(crlf));
    }
    buffers.push_back(boost::asio::buffer(crlf));
    return buffers;
}

inline Reply Reply::stockReply(Reply::status_type status) {
	Reply rep;
	rep.status = status;
	rep.content = ToString(status);
	rep.headers.resize(3);
	rep.headers[0].name = "Access-Control-Allow-Origin";
	rep.headers[0].value = "*";
	rep.headers[1].name = "Content-Length";

    std::string s;
    intToString(rep.content.size(), s);

    rep.headers[1].value = s;
	rep.headers[2].name = "Content-Type";
	rep.headers[2].value = "text/html";
	return rep;
}
} // namespace http

#endif //BASIC_DATASTRUCTURES_H
