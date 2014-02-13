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

#include "RequestParser.h"

namespace http {

RequestParser::RequestParser() : state_(method_start) { }

void RequestParser::Reset() { state_ = method_start; }

boost::tuple<boost::tribool, char*> RequestParser::Parse(
    Request& req,
    char* begin,
    char* end,
    http::CompressionType * compressionType
) {
    while (begin != end) {
        boost::tribool result = consume(req, *begin++, compressionType);
        if (result || !result){
            return boost::make_tuple(result, begin);
        }
    }
    boost::tribool result = boost::indeterminate;
    return boost::make_tuple(result, begin);
}

boost::tribool RequestParser::consume(
    Request& req, char input,
    http::CompressionType * compressionType
) {
    switch (state_) {
    case method_start:
        if (!isChar(input) || isCTL(input) || isTSpecial(input)) {
            return false;
        } else {
            state_ = method;
            return boost::indeterminate;
        }
    case method:
        if (input == ' ') {
            state_ = uri;
            return boost::indeterminate;
        } else if (!isChar(input) || isCTL(input) || isTSpecial(input)) {
            return false;
        } else {
            return boost::indeterminate;
        }
    case uri_start:
        if (isCTL(input)) {
            return false;
        } else {
            state_ = uri;
            req.uri.push_back(input);
            return boost::indeterminate;
        }
    case uri:
        if (input == ' ') {
            state_ = http_version_h;
            return boost::indeterminate;
        } else if (isCTL(input)) {
            return false;
        } else {
            req.uri.push_back(input);
            return boost::indeterminate;
        }
    case http_version_h:
        if (input == 'H') {
            state_ = http_version_t_1;
            return boost::indeterminate;
        } else {
            return false;
        }
    case http_version_t_1:
        if (input == 'T') {
            state_ = http_version_t_2;
            return boost::indeterminate;
        } else {
            return false;
        }
    case http_version_t_2:
        if (input == 'T') {
            state_ = http_version_p;
            return boost::indeterminate;
        } else {
            return false;
        }
    case http_version_p:
        if (input == 'P') {
            state_ = http_version_slash;
            return boost::indeterminate;
        } else {
            return false;
        }
    case http_version_slash:
        if (input == '/') {
            state_ = http_version_major_start;
            return boost::indeterminate;
        } else {
            return false;
        }
    case http_version_major_start:
        if (isDigit(input)) {
            state_ = http_version_major;
            return boost::indeterminate;
        } else {
            return false;
        }
    case http_version_major:
        if (input == '.') {
            state_ = http_version_minor_start;
            return boost::indeterminate;
        } else if (isDigit(input)) {
            return boost::indeterminate;
        } else {
            return false;
        }
    case http_version_minor_start:
        if (isDigit(input)) {
            state_ = http_version_minor;
            return boost::indeterminate;
        } else {
            return false;
        }
    case http_version_minor:
        if (input == '\r') {
            state_ = expecting_newline_1;
            return boost::indeterminate;
        } else if (isDigit(input)) {
            return boost::indeterminate;
        }
        else {
            return false;
        }
    case expecting_newline_1:
        if (input == '\n') {
            state_ = header_line_start;
            return boost::indeterminate;
        } else {
            return false;
        }
    case header_line_start:
        if(header.name == "Accept-Encoding") {
            /* giving gzip precedence over deflate */
            if(header.value.find("deflate") != std::string::npos)
                *compressionType = deflateRFC1951;
            if(header.value.find("gzip") != std::string::npos)
                *compressionType = gzipRFC1952;
        }

        if("Referer" == header.name)
            req.referrer = header.value;

        if("User-Agent" == header.name)
            req.agent = header.value;

        if (input == '\r') {
            state_ = expecting_newline_3;
            return boost::indeterminate;
        } else if (!isChar(input) || isCTL(input) || isTSpecial(input)) {
            return false;
        } else {
            state_ = header_name;
            header.Clear();
            header.name.push_back(input);
            return boost::indeterminate;
        }
    case header_lws:
        if (input == '\r') {
            state_ = expecting_newline_2;
            return boost::indeterminate;
        } else if (input == ' ' || input == '\t') {
            return boost::indeterminate;
        }
        else if (isCTL(input)) {
            return false;
        } else {
            state_ = header_value;
            return boost::indeterminate;
        }
    case header_name:
        if (input == ':') {
            state_ = space_before_header_value;
            return boost::indeterminate;
        } else if (!isChar(input) || isCTL(input) || isTSpecial(input)) {
            return false;
        } else {
            header.name.push_back(input);
            return boost::indeterminate;
        }
    case space_before_header_value:
        if (input == ' ') {
            state_ = header_value;
            return boost::indeterminate;
        } else {
            return false;
        }
    case header_value:
        if (input == '\r') {
            state_ = expecting_newline_2;
            return boost::indeterminate;
        } else if (isCTL(input)) {
            return false;
        } else {
            header.value.push_back(input);
            return boost::indeterminate;
        }
    case expecting_newline_2:
        if (input == '\n') {
            state_ = header_line_start;
            return boost::indeterminate;
        } else {
            return false;
        }
    case expecting_newline_3:
        return (input == '\n');
    default:
        return false;
    }
}

inline bool RequestParser::isChar(int c) {
    return c >= 0 && c <= 127;
}

inline bool RequestParser::isCTL(int c) {
    return (c >= 0 && c <= 31) || (c == 127);
}

inline bool RequestParser::isTSpecial(int c) {
    switch (c) {
    case '(': case ')': case '<': case '>': case '@':
    case ',': case ';': case ':': case '\\': case '"':
    case '/': case '[': case ']': case '?': case '=':
    case '{': case '}': case ' ': case '\t':
        return true;
    default:
        return false;
    }
}

inline bool RequestParser::isDigit(int c) {
    return c >= '0' && c <= '9';
}

}
