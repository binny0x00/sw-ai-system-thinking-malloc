/*
 * mm.c - Segregated free list + prev_alloc 비트 기반 malloc 구현
 *
 * [블록 구조]
 *   할당 블록:  헤더(4B) | 페이로드 | 패딩
 *   가용 블록:  헤더(4B) | prev_ptr(8B) | next_ptr(8B) | ... | 푸터(4B)
 *
 * [헤더 비트 구조]
 *   [ 크기(29비트) | prev_alloc(1비트) | alloc(1비트) ]
 *   - alloc     : 현재 블록 할당 여부
 *   - prev_alloc: 이전 블록 할당 여부 (할당된 블록은 푸터 생략)
 *
 * [Segregated free list - 크기 구간별 bin]
 *   bins[0] : 1    ~ 8
 *   bins[1] : 9    ~ 16
 *   bins[2] : 17   ~ 32
 *   bins[3] : 33   ~ 64
 *   bins[4] : 65   ~ 128
 *   bins[5] : 129  ~ 256
 *   bins[6] : 257  ~ 512
 *   bins[7] : 513  ~ 1024
 *   bins[8] : 1025 ~ 2048
 *   bins[9] : 2049 ~ 4096
 *   bins[10]: 4097 ~
 */
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <unistd.h>
#include <string.h>

#include "mm.h"
#include "memlib.h"

team_t team = {
    "ateam",
    "Harry Bovik",
    "bovik@cs.cmu.edu",
    "",
    ""};

/* ================================================================
 * 상수 및 매크로
 * ================================================================ */

#define WSIZE       4           /* 워드 크기 (bytes) */
#define DSIZE       8           /* 더블 워드 크기 (bytes) */
#define CHUNKSIZE   (1 << 12)   /* 힙 확장 단위 (bytes) */
#define ALIGNMENT   8           /* 정렬 기준 (bytes) */
#define NUM_CLASSES 11          /* segregated bin 개수 */

#define MAX(x, y)   ((x) > (y) ? (x) : (y))

/*
 * 가용 블록 최소 크기: header(4) + prev_ptr(8) + next_ptr(8) + footer(4) = 24바이트
 * 이보다 작으면 포인터를 쓸 공간이 없어 다음 블록을 덮어씀
 */
#define MIN_BLOCK_SIZE  (2*DSIZE + 2*WSIZE)

/*
 * PACK - size, prev_alloc, alloc을 하나의 워드로 패킹
 *   [ 크기(상위 29비트) | prev_alloc(bit1) | alloc(bit0) ]
 */
#define PACK(size, prev_alloc, alloc)   ((size) | ((prev_alloc) << 1) | (alloc))

/* 주소 p에서 워드 읽기/쓰기 */
#define GET(p)          (*(unsigned int *)(p))
#define PUT(p, val)     (*(unsigned int *)(p) = (val))

/* 포인터 읽기/쓰기 (가용 블록의 prev_ptr, next_ptr용) */
#define GET_PTR(p)      (*(char **)(p))
#define PUT_PTR(p, ptr) (*(char **)(p) = (ptr))

/* 주소 p의 헤더에서 크기, 할당 여부, 이전 블록 할당 여부 읽기 */
#define GET_SIZE(p)         (GET(p) & ~0x7)
#define GET_ALLOC(p)        (GET(p) & 0x1)
#define GET_PREV_ALLOC(p)   ((GET(p) >> 1) & 0x1)

/* 블록 포인터 bp로 헤더/푸터 주소 계산 */
#define HDRP(bp)    ((char *)(bp) - WSIZE)
#define FTRP(bp)    ((char *)(bp) + GET_SIZE(HDRP(bp)) - DSIZE)

/* 블록 포인터 bp로 이전/다음 블록 주소 계산 */
#define NEXT_BLKP(bp)   ((char *)(bp) + GET_SIZE(HDRP(bp)))
#define PREV_BLKP(bp)   ((char *)(bp) - GET_SIZE((char *)(bp) - DSIZE))

/* 가용 블록 내 prev_ptr, next_ptr 위치 (페이로드 시작 기준) */
#define PREV_PTR(bp)    ((char *)(bp))
#define NEXT_PTR(bp)    ((char *)(bp) + DSIZE)

/*
 * size를 ALIGNMENT(8)의 배수로 올림
 *   - (size + 7): 올림을 위한 여유분 추가
 *   - & ~7      : 하위 3비트를 0으로 → 8의 배수로 내림
 */
#define ALIGN(size) (((size) + (ALIGNMENT - 1)) & ~(ALIGNMENT - 1))

/* ================================================================
 * 전역 변수
 * ================================================================ */

/* segregated free list의 각 bin 헤드 포인터 배열 */
static char *seg_list[NUM_CLASSES];

/* ================================================================
 * 내부 함수 전방 선언
 * ================================================================ */

static int   get_class(size_t size);
static void  insert_free(void *bp);
static void  remove_free(void *bp);
static void *coalesce(void *bp);
static void *extend_heap(size_t words);
static void *find_fit(size_t asize);
static void  place(void *bp, size_t asize);

/* ================================================================
 * 내부 함수 구현
 * ================================================================ */

/*
 * get_class - size에 해당하는 segregated bin 인덱스 반환
 *
 * 8씩 배로 커지는 구간으로 나눔:
 *   ~8 → 0, ~16 → 1, ~32 → 2, ..., 4097~ → 10
 */
static int get_class(size_t size)
{
    int class = 0;
    size_t s = 8;
    while (class < NUM_CLASSES - 1 && size > s) {
        s <<= 1;
        class++;
    }
    return class;
}

/*
 * insert_free - 가용 블록 bp를 해당 bin의 맨 앞에 삽입 (LIFO)
 *
 * 이중 연결 리스트:
 *   bp의 prev_ptr → NULL
 *   bp의 next_ptr → 기존 헤드
 *   기존 헤드의 prev_ptr → bp
 *   bin 헤드 → bp
 */
static void insert_free(void *bp)
{
    int class = get_class(GET_SIZE(HDRP(bp)));
    char *head = seg_list[class];

    PUT_PTR(PREV_PTR(bp), NULL);
    PUT_PTR(NEXT_PTR(bp), head);

    if (head != NULL)
        PUT_PTR(PREV_PTR(head), bp);

    seg_list[class] = bp;
}

/*
 * remove_free - 가용 리스트에서 블록 bp 제거
 *
 * bp의 prev/next를 서로 연결하여 리스트에서 분리.
 * bp가 헤드였으면 bin 헤드를 next로 교체.
 */
static void remove_free(void *bp)
{
    int class = get_class(GET_SIZE(HDRP(bp)));
    char *prev = GET_PTR(PREV_PTR(bp));
    char *next = GET_PTR(NEXT_PTR(bp));

    if (prev != NULL)
        PUT_PTR(NEXT_PTR(prev), next);
    else
        seg_list[class] = next;

    if (next != NULL)
        PUT_PTR(PREV_PTR(next), prev);
}

/*
 * coalesce - 인접한 가용 블록과 병합
 *
 * prev_alloc 비트로 이전 블록 할당 여부 확인 (이전 블록 푸터 불필요).
 * 병합 후 다음 블록의 헤더에서 prev_alloc 비트를 0으로 업데이트.
 *
 * [Case 1] 이전/다음 모두 할당   → 병합 없음
 * [Case 2] 다음만 가용           → 현재 + 다음 병합
 * [Case 3] 이전만 가용           → 이전 + 현재 병합
 * [Case 4] 이전/다음 모두 가용   → 이전 + 현재 + 다음 병합
 */
static void *coalesce(void *bp)
{
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(bp));  /* 헤더의 prev_alloc 비트 */
    size_t next_alloc = GET_ALLOC(HDRP(NEXT_BLKP(bp)));
    size_t size = GET_SIZE(HDRP(bp));
    char *next_blk;

    if (prev_alloc && next_alloc) {                 /* Case 1 */
        PUT(HDRP(bp), PACK(size, 1, 0));
        PUT(FTRP(bp), PACK(size, 1, 0));
        /* 다음 블록의 prev_alloc 비트를 0으로 */
        next_blk = NEXT_BLKP(bp);
        PUT(HDRP(next_blk), PACK(GET_SIZE(HDRP(next_blk)), 0, GET_ALLOC(HDRP(next_blk))));
    }
    else if (prev_alloc && !next_alloc) {           /* Case 2: 다음 블록과 병합 */
        remove_free(NEXT_BLKP(bp));
        size += GET_SIZE(HDRP(NEXT_BLKP(bp)));
        PUT(HDRP(bp), PACK(size, 1, 0));
        PUT(FTRP(bp), PACK(size, 1, 0));
        /* 다음 블록의 prev_alloc 비트를 0으로 */
        next_blk = NEXT_BLKP(bp);
        PUT(HDRP(next_blk), PACK(GET_SIZE(HDRP(next_blk)), 0, GET_ALLOC(HDRP(next_blk))));
    }
    else if (!prev_alloc && next_alloc) {           /* Case 3: 이전 블록과 병합 */
        size_t pp_alloc = GET_PREV_ALLOC(HDRP(PREV_BLKP(bp)));
        remove_free(PREV_BLKP(bp));
        size += GET_SIZE(HDRP(PREV_BLKP(bp)));
        bp = PREV_BLKP(bp);
        PUT(HDRP(bp), PACK(size, pp_alloc, 0));
        PUT(FTRP(bp), PACK(size, pp_alloc, 0));
        /* 다음 블록의 prev_alloc 비트를 0으로 */
        next_blk = NEXT_BLKP(bp);
        PUT(HDRP(next_blk), PACK(GET_SIZE(HDRP(next_blk)), 0, GET_ALLOC(HDRP(next_blk))));
    }
    else {                                          /* Case 4: 이전/다음 모두 병합 */
        size_t pp_alloc = GET_PREV_ALLOC(HDRP(PREV_BLKP(bp)));
        remove_free(NEXT_BLKP(bp));
        remove_free(PREV_BLKP(bp));
        size += GET_SIZE(HDRP(PREV_BLKP(bp))) +
                GET_SIZE(HDRP(NEXT_BLKP(bp)));
        bp = PREV_BLKP(bp);
        PUT(HDRP(bp), PACK(size, pp_alloc, 0));
        PUT(FTRP(bp), PACK(size, pp_alloc, 0));
        /* 다음 블록의 prev_alloc 비트를 0으로 */
        next_blk = NEXT_BLKP(bp);
        PUT(HDRP(next_blk), PACK(GET_SIZE(HDRP(next_blk)), 0, GET_ALLOC(HDRP(next_blk))));
    }

    insert_free(bp);
    return bp;
}

/*
 * extend_heap - 힙을 words 워드 단위로 확장
 *
 * 정렬을 유지하기 위해 짝수 워드로 올림.
 * 확장된 영역을 새 가용 블록으로 초기화하고 coalesce 호출.
 * 새 블록의 prev_alloc은 기존 에필로그 헤더의 prev_alloc 비트를 따름.
 */
static void *extend_heap(size_t words)
{
    char *bp;
    size_t size;

    size = (words % 2) ? (words + 1) * WSIZE : words * WSIZE;
    if ((long)(bp = mem_sbrk(size)) == -1)
        return NULL;

    /* 기존 에필로그 헤더가 있던 자리가 새 블록의 헤더 위치 */
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(bp));

    PUT(HDRP(bp), PACK(size, prev_alloc, 0));   /* 새 가용 블록 헤더 */
    PUT(FTRP(bp), PACK(size, prev_alloc, 0));   /* 새 가용 블록 푸터 */
    PUT(HDRP(NEXT_BLKP(bp)), PACK(0, 0, 1));    /* 새 에필로그 헤더 */

    return coalesce(bp);
}

/*
 * find_fit - asize 이상의 가용 블록 탐색 (segregated first-fit)
 *
 * asize에 해당하는 bin부터 시작해 더 큰 bin 방향으로 탐색.
 * 각 bin 내에서 first-fit으로 탐색하여 첫 번째 적합 블록 반환.
 */
static void *find_fit(size_t asize)
{
    for (int i = get_class(asize); i < NUM_CLASSES; i++) {
        char *bp = seg_list[i];
        while (bp != NULL) {
            if (GET_SIZE(HDRP(bp)) >= asize)
                return bp;
            bp = GET_PTR(NEXT_PTR(bp));
        }
    }
    return NULL;
}

/*
 * place - bp 블록에 asize 바이트 할당
 *
 * 남은 공간이 최소 블록 크기(헤더 + prev_ptr + next_ptr + 푸터 = 4+8+8+4 = 24B)
 * 이상이면 분할. 할당된 블록은 푸터를 쓰지 않고,
 * 다음 블록의 헤더에서 prev_alloc 비트를 1로 업데이트.
 */
static void place(void *bp, size_t asize)
{
    size_t csize = GET_SIZE(HDRP(bp));
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(bp));

    remove_free(bp);

    /* 남은 공간이 MIN_BLOCK_SIZE(24B) 이상일 때만 분할 */
    if ((csize - asize) >= MIN_BLOCK_SIZE) {
        /* 분할: asize만큼 할당 (푸터 없음) */
        PUT(HDRP(bp), PACK(asize, prev_alloc, 1));

        /* 나머지를 새 가용 블록으로 (prev_alloc=1, alloc=0) */
        char *next = NEXT_BLKP(bp);
        PUT(HDRP(next), PACK(csize - asize, 1, 0));
        PUT(FTRP(next), PACK(csize - asize, 1, 0));

        /* 다다음 블록의 prev_alloc 비트를 0으로 */
        char *next2 = NEXT_BLKP(next);
        PUT(HDRP(next2), PACK(GET_SIZE(HDRP(next2)), 0, GET_ALLOC(HDRP(next2))));

        insert_free(next);
    } else {
        /* 분할 없이 전체 할당 (푸터 없음) */
        PUT(HDRP(bp), PACK(csize, prev_alloc, 1));

        /* 다음 블록의 prev_alloc 비트를 1로 */
        char *next = NEXT_BLKP(bp);
        size_t next_alloc = GET_ALLOC(HDRP(next));
        PUT(HDRP(next), PACK(GET_SIZE(HDRP(next)), 1, next_alloc));
        /* 다음 블록이 가용이면 푸터도 동기화 */
        if (!next_alloc)
            PUT(FTRP(next), PACK(GET_SIZE(HDRP(next)), 1, 0));
    }
}

/* ================================================================
 * 공개 함수 구현
 * ================================================================ */

/*
 * mm_init - 힙 초기화
 *
 * seg_list 배열을 NULL로 초기화.
 * 초기 힙 구조:
 *   [패딩(4B)][프롤로그 헤더(4B)][프롤로그 푸터(4B)][에필로그 헤더(4B)]
 *
 * 프롤로그/에필로그는 coalesce 경계 조건 처리를 단순화하기 위한 sentinel.
 * 이후 CHUNKSIZE만큼 힙을 확장하여 초기 가용 블록 생성.
 */
int mm_init(void)
{
    for (int i = 0; i < NUM_CLASSES; i++)
        seg_list[i] = NULL;

    char *heap_start;
    if ((heap_start = mem_sbrk(4 * WSIZE)) == (void *)-1)
        return -1;

    PUT(heap_start,              0);                    /* 정렬 패딩 */
    PUT(heap_start + (1*WSIZE),  PACK(DSIZE, 1, 1));    /* 프롤로그 헤더 */
    PUT(heap_start + (2*WSIZE),  PACK(DSIZE, 1, 1));    /* 프롤로그 푸터 */
    PUT(heap_start + (3*WSIZE),  PACK(0, 1, 1));        /* 에필로그 헤더 */

    if (extend_heap(CHUNKSIZE / WSIZE) == NULL)
        return -1;
    return 0;
}

/*
 * mm_malloc - size 바이트 할당
 *
 * 1. 헤더 오버헤드(WSIZE)와 정렬을 포함한 asize 계산
 *    - 최소 블록: 헤더(4B) + prev_ptr(8B) + next_ptr(8B) + 푸터(4B) = 24B
 *      → 최소 2*DSIZE(16B) 보장 (가용 상태 기준)
 * 2. find_fit으로 가용 블록 탐색
 * 3. 없으면 힙 확장 후 할당
 */
void *mm_malloc(size_t size)
{
    size_t asize;
    size_t extendsize;
    char *bp;

    if (size == 0)
        return NULL;

    /*
     * 헤더(4B) + 페이로드를 8바이트 정렬.
     * 단, 가용 상태가 되었을 때 prev_ptr/next_ptr을 담을 수 있도록
     * MIN_BLOCK_SIZE(24B) 이상을 보장.
     */
    asize = MAX(ALIGN(size + WSIZE), MIN_BLOCK_SIZE);

    if ((bp = find_fit(asize)) != NULL) {
        place(bp, asize);
        return bp;
    }

    extendsize = MAX(asize, CHUNKSIZE);
    if ((bp = extend_heap(extendsize / WSIZE)) == NULL)
        return NULL;
    place(bp, asize);
    return bp;
}

/*
 * mm_free - ptr 블록 해제
 *
 * 헤더의 alloc 비트를 0으로 설정하고 푸터를 추가.
 * 이전 블록 할당 여부는 헤더의 prev_alloc 비트에서 읽어 유지.
 * 이후 coalesce로 인접 가용 블록과 병합.
 */
void mm_free(void *ptr)
{
    size_t size = GET_SIZE(HDRP(ptr));
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(ptr));

    PUT(HDRP(ptr), PACK(size, prev_alloc, 0));  /* 헤더: 가용으로 변경 */
    PUT(FTRP(ptr), PACK(size, prev_alloc, 0));  /* 푸터 추가 */

    coalesce(ptr);
}

/*
 * mm_realloc - ptr 블록을 size 바이트로 재할당
 *
 * - ptr == NULL : mm_malloc(size)와 동일
 * - size == 0   : mm_free(ptr)와 동일
 * - 축소 요청  : 현재 블록 그대로 반환 (공간 낭비 없음)
 * - 확장 요청  : 다음 블록이 가용이고 합친 크기가 충분하면 제자리 확장.
 *               그렇지 않으면 새 블록 할당 → 복사 → 기존 해제.
 *
 * [제자리 확장 조건]
 *   현재 블록 크기 + 다음 가용 블록 크기 >= asize
 *
 * [제자리 확장 효과]
 *   realloc-bal.rep처럼 점진적으로 크기가 커지는 패턴에서
 *   malloc+copy+free 대신 블록을 그 자리에서 넓혀 단편화 방지.
 */
void *mm_realloc(void *ptr, size_t size)
{
    if (ptr == NULL)
        return mm_malloc(size);

    if (size == 0) {
        mm_free(ptr);
        return NULL;
    }

    size_t asize = MAX(ALIGN(size + WSIZE), MIN_BLOCK_SIZE);
    size_t cur_size = GET_SIZE(HDRP(ptr));

    /* 축소 또는 현재 크기로 충분한 경우: 그대로 반환 */
    if (asize <= cur_size)
        return ptr;

    /* 다음 블록이 가용이고 합쳤을 때 충분한지 확인 */
    char *next = NEXT_BLKP(ptr);
    size_t next_alloc = GET_ALLOC(HDRP(next));
    size_t next_size = GET_SIZE(HDRP(next));
    size_t combined = cur_size + next_size;

    if (!next_alloc && combined >= asize) {
        /* 제자리 확장: 다음 가용 블록을 흡수 */
        remove_free(next);

        size_t prev_alloc = GET_PREV_ALLOC(HDRP(ptr));

        if ((combined - asize) >= MIN_BLOCK_SIZE) {
            /* 남은 공간으로 새 가용 블록 분할 */
            PUT(HDRP(ptr), PACK(asize, prev_alloc, 1));
            char *new_next = NEXT_BLKP(ptr);
            PUT(HDRP(new_next), PACK(combined - asize, 1, 0));
            PUT(FTRP(new_next), PACK(combined - asize, 1, 0));
            /* 다다음 블록의 prev_alloc 비트를 0으로 */
            char *new_next2 = NEXT_BLKP(new_next);
            PUT(HDRP(new_next2), PACK(GET_SIZE(HDRP(new_next2)), 0, GET_ALLOC(HDRP(new_next2))));
            insert_free(new_next);
        } else {
            /* 남은 공간이 작으면 전체 흡수 */
            PUT(HDRP(ptr), PACK(combined, prev_alloc, 1));
            /* 다음 블록의 prev_alloc 비트를 1로 */
            char *new_next = NEXT_BLKP(ptr);
            size_t na = GET_ALLOC(HDRP(new_next));
            PUT(HDRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, na));
            if (!na)
                PUT(FTRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, 0));
        }
        return ptr;
    }

    /* 제자리 확장 불가 → 새 블록 할당 후 복사 */
    void *newptr = mm_malloc(size);
    if (newptr == NULL)
        return NULL;

    size_t copySize = cur_size - WSIZE;
    if (size < copySize)
        copySize = size;
    memcpy(newptr, ptr, copySize);

    mm_free(ptr);
    return newptr;
}
