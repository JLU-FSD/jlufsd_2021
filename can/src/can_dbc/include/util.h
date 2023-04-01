#ifndef _UTIL_H_
#define _UTIL_H_

#include <string>
#include <vector>

void split(const std::string &s, const std::string &seperator, std::vector<std::string> *result);
void getAllFilesInFolder(const std::string &dir_in, std::vector<std::string> *files);

#endif // _UTIL_H_
