/*
 * memlib.c - a module that simulates the memory system.  Needed because it
 *            allows us to interleave calls from the student's malloc package
 *            with the system's malloc package in libc.
 */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <sys/mman.h>
#include <string.h>
#include <errno.h>

#include "memlib.h"
#include "config.h"

/* private variables */
static char *mem_start_brk; /* points to first byte of heap */
static char *mem_brk;       /* points to last byte of heap */
static char *mem_max_addr;  /* largest legal heap address */

/*
 * mem_init - initialize the memory system model
 * 프로그램 시작 시 20MB 크기의 메모리 블록을 미리 확보하고 그 안에서 힙 시뮬레이션
 * 확보한 메모리의 시작 주소를 mem_start_brk에 저장
 * malloc은 C 표준에 의해 연속된 메모리 블록을 반환하도록 보장됨.
 * 이것이 가능한 이유는 OS가 가상 메모리를 사용하기 때문에, 물리적으로 분산되어 있어도 가상 주소 공간에서는 연속된 것처럼 매핑해줄 수 있기 때문.
 */
void mem_init(void)
{
    /* allocate the storage we will use to model the available VM */
    if ((mem_start_brk = (char *)malloc(MAX_HEAP)) == NULL)
    {
        fprintf(stderr, "mem_init_vm: malloc error\n");
        exit(1);
    }

    mem_max_addr = mem_start_brk + MAX_HEAP; /* max legal heap address */
    mem_brk = mem_start_brk;                 /* heap is empty initially */
}

/*
 * mem_deinit - free the storage used by the memory system model
 */
void mem_deinit(void)
{
    free(mem_start_brk);
}

/*
 * mem_reset_brk - reset the simulated brk pointer to make an empty heap
 */
void mem_reset_brk()
{
    mem_brk = mem_start_brk;
}

/*
 * mem_sbrk - simple model of the sbrk function. Extends the heap
 *    by incr bytes and returns the start address of the new area. In
 *    this model, the heap cannot be shrunk.
 *
 * 실제 OS의 sbrk() 시스템 콜을 흉내낸 함수.
 * incr 바이트만큼 힙을 확장하고, 확장 전 brk 포인터(새로 할당된 영역의 시작 주소)를 반환한다.
 *
 * [동작 흐름]
 *  1. 현재 brk 위치를 old_brk에 저장
 *  2. incr가 음수이거나 확장 후 brk가 힙 최대 주소를 초과하면
 *     errno를 ENOMEM으로 설정하고 (void *)-1 반환 (실패)
 *  3. 정상이면 mem_brk를 incr만큼 앞으로 이동시키고 old_brk 반환 (성공)
 *
 * [주의] 이 시뮬레이터에서는 힙을 줄이는 것(incr < 0)은 허용하지 않는다.
 */
void *mem_sbrk(int incr)
{
    char *old_brk = mem_brk;

    if ((incr < 0) || ((mem_brk + incr) > mem_max_addr))
    {
        errno = ENOMEM;
        fprintf(stderr, "ERROR: mem_sbrk failed. Ran out of memory...\n");
        return (void *)-1;
    }
    mem_brk += incr;
    return (void *)old_brk;
}

/*
 * mem_heap_lo - return address of the first heap byte
 */
void *mem_heap_lo()
{
    return (void *)mem_start_brk;
}

/*
 * mem_heap_hi - return address of last heap byte
 */
void *mem_heap_hi()
{
    return (void *)(mem_brk - 1);
}

/*
 * mem_heapsize() - returns the heap size in bytes
 */
size_t mem_heapsize()
{
    return (size_t)(mem_brk - mem_start_brk);
}

/*
 * mem_pagesize() - returns the page size of the system
 */
size_t mem_pagesize()
{
    return (size_t)getpagesize();
}
