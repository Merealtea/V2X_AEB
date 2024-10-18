#ifndef IMAGE_PIPELINE__COMMON_HPP_
#define IMAGE_PIPELINE__COMMON_HPP_

#ifdef _WIN32
#include <process.h>
#define GETPID _getpid
#else
#include <unistd.h>
#define GETPID getpid
#endif

#include <string>
#include <sys/shm.h>
#include <iostream>
#include <fstream>
#include "interprocess.hpp"

using namespace std;

// Define the shared memory structure for single camera
#define TARGETWIDTH 640
#define TARGETHEIGHT 368
#define RESIZED_IMG_SIZE TARGETWIDTH * TARGETHEIGHT * 3

typedef struct{
   int32_t camera_id {0};
   int32_t frame_idx {0};
   int32_t width {TARGETWIDTH};
   int32_t height {TARGETHEIGHT};
   int32_t num_channel {3};
   int32_t width_no_pad;
   int32_t height_no_pad;
   int32_t original_width;
   int32_t original_height;
   float32_t ratio;
   double stamp;
   char data[RESIZED_IMG_SIZE] {};
} IMAGE_ELEMENT;

void allocate_shm(string &shm_name, int32_t byte_size, IMAGE_ELEMENT* &image_ptr)
{
  SharedMemory* shm_ = new SharedMemory(shm_name.data(), byte_size, true);

  if (shm_->fail())
  {
    cerr << "Failed to create shared memory" << endl;
    exit(1);
  }
  image_ptr = (IMAGE_ELEMENT*)shm_->read();
  printf("Shared memory %s created at %p\n", shm_name.data(), image_ptr);
}


#endif // IMAGE_PIPELINE__COMMON_HPP_