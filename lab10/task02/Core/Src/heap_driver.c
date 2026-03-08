#include "heap_driver.h"
#include "stm32f3xx_hal_uart.h"
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define HEAP_START_ADDR  ((uint8_t*)0x20001000)
#define HEAP_SIZE        (4 * 1024)
#define BLOCK_SIZE       16
#define BLOCK_COUNT      (HEAP_SIZE / BLOCK_SIZE)   // 256
uint8_t block_map[BLOCK_COUNT]; // 0 = free, 1 = used
#define HEAP_END_ADDR ((uint8_t*)0x20001000 + 4096)
// Students should be provided the above code (includes and defines) and the function declarations in this file.
// They can figure out the rest.

// Allocation bitmap: 0 = free, 1 = used

// Add you code below
void heap_init(void){
    for (int i = 0; i < BLOCK_COUNT; i++){
        block_map[i] = 0;
    }
}

void* heap_alloc(size_t size){
    int blocks = size / BLOCK_SIZE;
    int count = 0;
    int i = 0;
    for (i = 0; i < BLOCK_COUNT; i++){
        if (count == blocks) break;

        if (block_map[i] == 0) count ++;  
        else count = 0;    // Ensure that we only count contiguous free blocks. If we encounter a used block, reset the count.
    }
    
    if (count != 0){
        uint8_t* address = ( (i - blocks) * 16 )+  HEAP_START_ADDR;   // starting block address = (block index * block size) + heap start address
        for (int j = i - blocks ; j < i ; j++){
            block_map[j] = 1;   // Marks the blocks as used in the block map. 
        }
        return address;
    }
    else return NULL;
}

void heap_free(void* ptr){
    uint8_t* p = ptr;
    if (ptr == NULL || p > HEAP_END_ADDR || p < HEAP_START_ADDR) return; // Boundary Checks

    int block_num = p - HEAP_START_ADDR;

    for (int i = block_num; i < BLOCK_COUNT; i++){
        block_map[i] = 0;    // Marks the blocks as free in the block map. 
    }
}