
#include "proto_file.h"
#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <glob.h>
#include <limits.h>
#include <stddef.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#include <fstream>
#include <string>

namespace common_tool
{

using std::istreambuf_iterator;
using std::string;
using std::vector;

bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         int file_descriptor)
{
    using google::protobuf::TextFormat;
    using google::protobuf::io::FileOutputStream;
    using google::protobuf::io::ZeroCopyOutputStream;
    if (file_descriptor < 0)
    {
        std::cerr << "Invalid file descriptor." << std::endl;
        return false;
    }
    ZeroCopyOutputStream *output = new FileOutputStream(file_descriptor);
    bool success = TextFormat::Print(message, output);
    delete output;
    close(file_descriptor);
    return success;
}

bool SetProtoToASCIIFile(const google::protobuf::Message &message,
                         const std::string &file_name)
{
    int fd = open(file_name.c_str(), O_WRONLY | O_CREAT | O_TRUNC, S_IRWXU);
    if (fd < 0)
    {
        std::cerr << "Unable to open file " << file_name << " to write." << std::endl;
        return false;
    }
    return SetProtoToASCIIFile(message, fd);
}

bool GetProtoFromASCIIFile(const std::string &file_name,
                           google::protobuf::Message *message)
{
    using google::protobuf::TextFormat;
    using google::protobuf::io::FileInputStream;
    using google::protobuf::io::ZeroCopyInputStream;
    int file_descriptor = open(file_name.c_str(), O_RDONLY);
    if (file_descriptor < 0)
    {
        std::cerr << "Failed to open file " << file_name << " in text mode." << std::endl;
        // Failed to open;
        return false;
    }

    ZeroCopyInputStream *input = new FileInputStream(file_descriptor);
    bool success = TextFormat::Parse(input, message);
    if (!success)
    {
        std::cerr << "Failed to parse file " << file_name << " as text proto." << std::endl;
    }
    delete input;
    close(file_descriptor);
    return success;
}

bool SetProtoToBinaryFile(const google::protobuf::Message &message,
                          const std::string &file_name)
{
    std::fstream output(file_name,
                        std::ios::out | std::ios::trunc | std::ios::binary);
    return message.SerializeToOstream(&output);
}

bool GetProtoFromBinaryFile(const std::string &file_name,
                            google::protobuf::Message *message)
{
    std::fstream input(file_name, std::ios::in | std::ios::binary);
    if (!input.good())
    {
        std::cerr << "Failed to open file " << file_name << " in binary mode." << std::endl;
        return false;
    }
    if (!message->ParseFromIstream(&input))
    {
        std::cerr << "Failed to parse file " << file_name << " as binary proto." << std::endl;
        return false;
    }
    return true;
}

bool GetProtoFromFile(const std::string &file_name,
                      google::protobuf::Message *message)
{
    // Try the binary parser first if it's much likely a binary proto.
    static const std::string kBinExt = ".bin";
    if (std::equal(kBinExt.rbegin(), kBinExt.rend(), file_name.rbegin()))
    {
        return GetProtoFromBinaryFile(file_name, message) ||
               GetProtoFromASCIIFile(file_name, message);
    }

    return GetProtoFromASCIIFile(file_name, message) ||
           GetProtoFromBinaryFile(file_name, message);
}

} // namespace common_tool
