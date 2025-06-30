// igc_parser.h
// 07.11.2023 ASR  First version

#include <map>

#ifndef __IGC_PARSER_H__
#define __IGC_PARSER_H__ 1

class IgcPosition {
public:
    int timestamp; // 080058
    double lat; // 47.1234
    double lon; //  8.1234
    double altGps; // 561.0
    double altBaro; // 512.0
};

class IgcFile {
public:
    int timestampMin;
    int timestampMax;
    double latMin;
    double latMax;
    double lonMin;
    double lonMax;
    std::map<int, IgcPosition*> positions;
};

class IgcSet {
public:
    double getLatMin();
    double getLatMax();
    double getLonMin();
    double getLonMax();

public:
    std::map<std::string, IgcFile*> files;
};

class IgcParser {
public:
    static int parseIgcFile(std::string path, IgcFile** file);
};

#endif // __IGC_PARSER_H__
