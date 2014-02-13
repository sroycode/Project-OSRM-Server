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

#include "../Util/GitDescription.h"
#include "../Util/OSRMException.h"
#include "../Util/SimpleLogger.h"
#include "../Util/TimingUtil.h"

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>
#include <boost/ref.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fcntl.h>
#ifdef __linux__
#include <malloc.h>
#endif
#include <algorithm>
#include <iomanip>
#include <numeric>
#include <vector>

const unsigned number_of_elements = 268435456;

struct Statistics { double min, max, med, mean, dev; };

void RunStatistics(std::vector<double> & timings_vector, Statistics & stats) {
    std::sort(timings_vector.begin(), timings_vector.end());
    stats.min = timings_vector.front();
    stats.max = timings_vector.back();
    stats.med = timings_vector[timings_vector.size()/2];
    double primary_sum =    std::accumulate(
                                timings_vector.begin(),
                                timings_vector.end(),
                                0.0
                            );
    stats.mean = primary_sum / timings_vector.size();

    double primary_sq_sum = std::inner_product( timings_vector.begin(),
                                timings_vector.end(),
                                timings_vector.begin(),
                                0.0
                            );
     stats.dev = std::sqrt(
        primary_sq_sum / timings_vector.size() - (stats.mean * stats.mean)
    );
}

int main (int argc, char * argv[]) {
    LogPolicy::GetInstance().Unmute();

        SimpleLogger().Write() <<
            "starting up engines, " << g_GIT_DESCRIPTION << ", " <<
            "compiled at " << __DATE__ << ", " __TIME__;

    if( 1 == argc ) {
        SimpleLogger().Write(logWARNING) <<
            "usage: " << argv[0] << " /path/on/device";
        return -1;
    }

    boost::filesystem::path test_path = boost::filesystem::path(argv[1]);
    test_path /= "osrm.tst";
    SimpleLogger().Write(logDEBUG) << "temporary file: " << test_path.string();

    try {
        //create files for testing
        if( 2 == argc) {
            //create file to test
            if( boost::filesystem::exists(test_path) ) {
                throw OSRMException("Data file already exists");
            }

            double time1, time2;
            int * random_array = new int[number_of_elements];
            std::generate (
                random_array,
                random_array+number_of_elements,
                std::rand
            );
#ifdef __APPLE__
            FILE * fd = fopen(test_path.string().c_str(), "w");
            fcntl(fileno(fd), F_NOCACHE, 1);
            fcntl(fileno(fd), F_RDAHEAD, 0);
            time1 = get_timestamp();
            write(
                fileno(fd),
                (char*)random_array,
                number_of_elements*sizeof(unsigned)
            );
            time2 = get_timestamp();
            fclose(fd);
#endif
#ifdef __linux__
            int f = open(
                test_path.string().c_str(),
                O_CREAT|O_TRUNC|O_WRONLY|O_SYNC,
                S_IRWXU
            );
            time1 = get_timestamp();
            int ret = write(
                f,
                random_array,
                number_of_elements*sizeof(unsigned)
            );
            if(-1 == ret) {
                throw OSRMException("could not write random data file");
            }
            time2 = get_timestamp();
            close(f);
#endif
            delete[] random_array;
            SimpleLogger().Write(logDEBUG) <<
                "writing raw 1GB took " << (time2-time1)*1000 << "ms";
            SimpleLogger().Write() << "raw write performance: " <<
                std::setprecision(5) << std::fixed <<
                1024*1024/((time2-time1)*1000) << "MB/sec";

            SimpleLogger().Write(logDEBUG) <<
                "finished creation of random data. Flush disk cache now!";

        } else {

            //
            // Run Non-Cached I/O benchmarks
            //

            if( !boost::filesystem::exists(test_path) ) {
                throw OSRMException("data file does not exist");
            }

            double time1, time2;
            //volatiles do not get optimized
            Statistics stats;

#ifdef __APPLE__
            volatile unsigned single_block[1024];
            char * raw_array = new char[number_of_elements*sizeof(unsigned)];
            FILE * fd = fopen(test_path.string().c_str(), "r");
            fcntl(fileno(fd), F_NOCACHE, 1);
            fcntl(fileno(fd), F_RDAHEAD, 0);
#endif
#ifdef __linux__
            char * single_block = (char*) memalign(
                512,
                1024*sizeof(unsigned)
            );

            int f = open(test_path.string().c_str(), O_RDONLY|O_DIRECT|O_SYNC);
            SimpleLogger().Write(logDEBUG) <<
                "opened, error: " << strerror(errno);
            char * raw_array = (char*) memalign(
                512,
                number_of_elements*sizeof(unsigned)
            );
#endif
            time1 = get_timestamp();
#ifdef __APPLE__
            read(fileno(fd), raw_array, number_of_elements*sizeof(unsigned));
            close(fileno(fd));
            fd = fopen(test_path.string().c_str(), "r");
#endif
#ifdef __linux__
            int ret = read(f, raw_array, number_of_elements*sizeof(unsigned));
            SimpleLogger().Write(logDEBUG) <<
                "read " << ret << " bytes, error: " << strerror(errno);
            close(f);
            f = open(test_path.string().c_str(), O_RDONLY|O_DIRECT|O_SYNC);
            SimpleLogger().Write(logDEBUG) <<
                "opened, error: " << strerror(errno);
#endif
            time2 = get_timestamp();

            SimpleLogger().Write(logDEBUG) <<
                "reading raw 1GB took " << (time2-time1)*1000 << "ms";
            SimpleLogger().Write() << "raw read performance: " <<
                std::setprecision(5) << std::fixed <<
                1024*1024/((time2-time1)*1000) << "MB/sec";

            std::vector<double> timing_results_raw_random;
            SimpleLogger().Write(logDEBUG) << "running 1000 random I/Os of 4KB";

#ifdef __APPLE__
            fseek(fd, 0, SEEK_SET);
#endif
#ifdef __linux__
            lseek(f, 0, SEEK_SET);
#endif
            //make 1000 random access, time each I/O seperately
            unsigned number_of_blocks = (number_of_elements*sizeof(unsigned)-1)/4096;
            for(unsigned i = 0; i < 1000; ++i) {
                unsigned block_to_read = std::rand()%number_of_blocks;
                off_t current_offset = block_to_read*4096;
                time1 = get_timestamp();
#ifdef __APPLE__
                int ret1 = fseek(fd, current_offset, SEEK_SET);
                int ret2 = read(fileno(fd), (char*)&single_block[0], 4096);
#endif
#ifdef __linux__
                int ret1 = lseek(f, current_offset, SEEK_SET);
                int ret2 = read(f, (char*)single_block, 4096);
#endif
                time2 = get_timestamp();
                if( ((off_t)-1) == ret1) {
                    SimpleLogger().Write(logWARNING)
                        << "offset: " << current_offset;
                    SimpleLogger().Write(logWARNING)
                        << "seek error " << strerror(errno);
                    throw OSRMException("seek error");
                }
                 if(-1 == ret2) {
                    SimpleLogger().Write(logWARNING)
                        << "offset: " << current_offset;
                    SimpleLogger().Write(logWARNING)
                        << "read error " << strerror(errno);
                    throw OSRMException("read error");
                }
               timing_results_raw_random.push_back((time2-time1)*1000.);
            }

            // Do statistics
            SimpleLogger().Write(logDEBUG) << "running raw random I/O statistics";
            std::ofstream random_csv("random.csv", std::ios::trunc);
            for(unsigned i = 0; i < timing_results_raw_random.size(); ++i) {
                random_csv << i << ", " << timing_results_raw_random[i] << std::endl;
            }
            random_csv.close();
            RunStatistics(timing_results_raw_random, stats);

            SimpleLogger().Write() << "raw random I/O: "  <<
                std::setprecision(5) << std::fixed <<
                "min: "  << stats.min  << "ms, " <<
                "mean: " << stats.mean << "ms, " <<
                "med: "  << stats.med  << "ms, " <<
                "max: "  << stats.max  << "ms, " <<
                "dev: "  << stats.dev  << "ms";

            std::vector<double> timing_results_raw_seq;
#ifdef __APPLE__
            fseek(fd, 0, SEEK_SET);
#endif
#ifdef __linux__
            lseek(f, 0, SEEK_SET);
#endif

            //read every 100th block
            for(
                unsigned i = 0;
                i < 1000;
                ++i
            ) {
                off_t current_offset = i*4096;
                time1 = get_timestamp();
    #ifdef __APPLE__
                int ret1 = fseek(fd, current_offset, SEEK_SET);
                int ret2 = read(fileno(fd), (char*)&single_block, 4096);
    #endif
    #ifdef __linux__
                int ret1 = lseek(f, current_offset, SEEK_SET);

                int ret2 = read(f, (char*)single_block, 4096);
    #endif
                time2 = get_timestamp();
                if( ((off_t)-1) == ret1) {
                    SimpleLogger().Write(logWARNING)
                        << "offset: " << current_offset;
                    SimpleLogger().Write(logWARNING)
                        << "seek error " << strerror(errno);
                    throw OSRMException("seek error");
                }
                 if(-1 == ret2) {
                    SimpleLogger().Write(logWARNING)
                        << "offset: " << current_offset;
                    SimpleLogger().Write(logWARNING)
                        << "read error " << strerror(errno);
                    throw OSRMException("read error");
                }
               timing_results_raw_seq.push_back((time2-time1)*1000.);
            }
    #ifdef __APPLE__
            fclose(fd);
            // free(single_element);
            free(raw_array);
            // free(single_block);
    #endif
    #ifdef __linux__
            close(f);
    #endif
            //Do statistics
            SimpleLogger().Write(logDEBUG) << "running sequential I/O statistics";
            //print simple statistics: min, max, median, variance
            std::ofstream seq_csv("sequential.csv", std::ios::trunc);
            for(unsigned i = 0; i < timing_results_raw_seq.size(); ++i) {
                seq_csv << i << ", " << timing_results_raw_seq[i] << std::endl;
            }
            seq_csv.close();
            RunStatistics(timing_results_raw_seq, stats);
            SimpleLogger().Write() << "raw sequential I/O: " <<
                std::setprecision(5) << std::fixed <<
                "min: "  << stats.min  << "ms, " <<
                "mean: " << stats.mean << "ms, " <<
                "med: "  << stats.med  << "ms, " <<
                "max: "  << stats.max  << "ms, " <<
                "dev: "  << stats.dev  << "ms";

            if( boost::filesystem::exists(test_path) ) {
                boost::filesystem::remove(test_path);
                SimpleLogger().Write(logDEBUG) << "removing temporary files";
            }
        }
    } catch ( const std::exception & e ) {
        SimpleLogger().Write(logWARNING) << "caught exception: " << e.what();
        SimpleLogger().Write(logWARNING) << "cleaning up, and exiting";
        if(boost::filesystem::exists(test_path)) {
            boost::filesystem::remove(test_path);
            SimpleLogger().Write(logWARNING) << "removing temporary files";
        }
        return -1;
    }
    return 0;
}
