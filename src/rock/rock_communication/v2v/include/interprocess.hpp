#ifndef INTERPROCESS_HPP
#define INTERPROCESS_HPP

#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <cstring>

class SharedMemory {
    public:
        SharedMemory(const char* name="shmem", size_t size=0, bool create=true);
        ~SharedMemory();

        void write(const char* data, size_t size);
        const char* read();
        size_t get_size();
        const char* get_name();
        int get_fd();
        bool fail();  // call this for proper error handling of shared memory creation
    
    private:
        void* buffer;  // memory address
        size_t size;  // bytes
        const char* name;  // name of shared memory
        int fd;  // file descriptor
        bool unsuccessful = false;  // safety flag to ensure proper clean up of memory
};

// Creates shared memory which is used for both read and write
// Set "create flag" to false to access an existing shared memory
SharedMemory::SharedMemory(const char* name, size_t size, bool create) {
    this->name = name;
    this->size = size;
    
    // Set up file descriptor for shared memory
    if (create) {
        this->fd = shm_open(this->name, O_CREAT|O_RDWR, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);
    }
    else {
        this->fd = shm_open(this->name, O_RDWR, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP);
    }

    if (this->fd == -1) {
         perror("shm_open");
         this->unsuccessful = true;
    }

    // configure the size of the shared memory
     if (ftruncate(this->fd, this->size) == -1) {
        perror("ftruncate");
         this->unsuccessful = true;
    }

    // Creating the memory address for the shared memory
    this->buffer = mmap(0, this->size, PROT_WRITE | PROT_READ, MAP_SHARED, this->fd, 0);

    if (this->buffer == MAP_FAILED) {
        perror("mmap");
        this->unsuccessful = true;
    }
    

    std::cout << "SUCCESSFULLY SETUP SHARED MEMORY :: <<NAME :: " << this->name << "; SIZE :: " << this->size << ">>\n";
}

SharedMemory::~SharedMemory() {
    shm_unlink(this->name);
}

// Write a value to the shared memory. String type supported at the moment
void SharedMemory::write(const char* data, size_t size) {
    memcpy(this->buffer, data, size);
}

// Read from the shared memory
const char* SharedMemory::read() {
    return (char*)this->buffer;
}

size_t SharedMemory::get_size() {
    return this->size;
}

const char* SharedMemory::get_name() {
    return this->name;
}

int SharedMemory::get_fd() {
    return this->fd;
}

bool SharedMemory::fail() {
    return this->unsuccessful;
}

// Shared memory exclusive process controller
namespace PROCESS {
    void wait(SharedMemory* shm, std::string criteria);
}

// Hold the program until the criteria is met
void PROCESS::wait(SharedMemory* shm, std::string criteria) {
    bool loop = true;
    while (loop) {
        if (std::string(shm->read()) == criteria) {
            loop = false;
        }
    }
}

#endif // INTERPROCESS_HPP