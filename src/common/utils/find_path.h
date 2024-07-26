//
// Created by luyifan on 19-6-10.
//

#pragma once

#include <unistd.h>

#include <cstring>
#include <string>

inline size_t GetCurrentExcutableFilePathName(char* processdir,
                                              char* processname, size_t len) {
  char* path_end;
  if (readlink("/proc/self/exe", processdir, len) <= 0) return -1;
  path_end = strrchr(processdir, '/');
  if (path_end == NULL) return -1;
  ++path_end;
  strcpy(processname, path_end);
  *path_end = '\0';
  return (size_t)(path_end - processdir);
}

inline std::string expand_catkin_ws(std::string path) {
  char buf1[100];
  char buf2[100];

  GetCurrentExcutableFilePathName(buf1, buf2, 100);
  std::string string_buf = buf1;

  return string_buf + "../../../" + path;
}
