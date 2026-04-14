/*
 * mm.c - Segregated free list + prev_alloc 비트 기반 malloc 구현
 *
 * [블록 구조]
 *   할당 블록:  헤더(4B) | 페이로드 | 패딩
 *   가용 블록:  헤더(4B) | prev_offset(4B) | next_offset(4B) | ... | 푸터(4B)
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
 * [가용 블록 포인터 저장 방식 - 힙 오프셋]
 *
 * 64비트 환경에서도 8바이트 포인터 대신 4바이트 힙 오프셋을 저장.
 * 힙 최대 크기가 20MB이므로 4바이트(최대 4GB)로 충분히 표현 가능.
 * offset = 0 → NULL 표현
 *   (실제 free 블록의 최소 offset = 16으로, 0과 충돌하지 않음)
 *
 * MIN_BLOCK_SIZE = header(4) + prev_offset(4) + next_offset(4) + footer(4) = 16바이트
 */
#define PTR_SIZE 4

static inline char *_get_ptr(void *p) {
    unsigned int offset = *(unsigned int *)p;
    return offset == 0 ? NULL : (char *)mem_heap_lo() + offset;
}
static inline void _put_ptr(void *p, void *ptr) {
    *(unsigned int *)p = (ptr == NULL)
        ? 0
        : (unsigned int)((char *)ptr - (char *)mem_heap_lo());
}
#define GET_PTR(p)      _get_ptr(p)
#define PUT_PTR(p, ptr) _put_ptr(p, (void *)(ptr))

/*
 * 가용 블록 최소 크기: header(4) + prev_offset(4) + next_offset(4) + footer(4) = 16바이트
 */
#define MIN_BLOCK_SIZE  (2*WSIZE + 2*PTR_SIZE)

/*
 * PACK - size, prev_alloc, alloc을 하나의 워드로 패킹
 *   [ 크기(상위 29비트) | prev_alloc(bit1) | alloc(bit0) ]
 */
#define PACK(size, prev_alloc, alloc)   ((size) | ((prev_alloc) << 1) | (alloc))

/* 주소 p에서 워드 읽기/쓰기 */
#define GET(p)          (*(unsigned int *)(p))
#define PUT(p, val)     (*(unsigned int *)(p) = (val))

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
#define NEXT_PTR(bp)    ((char *)(bp) + PTR_SIZE)

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
 * 프로파일링 (make profile 로 빌드 시 활성화)
 *
 * 측정 항목:
 *   bin_insert_count[i] : bin i에 삽입된 횟수
 *   bin_length_sum[i]   : 삽입 시점의 bin i 리스트 길이 누적합
 *
 * → 평균 리스트 길이 = bin_length_sum / bin_insert_count
 *   이 값이 클수록 정렬 삽입(O(n))의 비용이 높음.
 *   반대로 크기 분산이 클수록 정렬의 util 개선 효과가 큼.
 *   두 값을 비교해 임계 class를 데이터로 결정할 수 있음.
 * ================================================================ */
#ifdef PROFILE
static size_t    bin_insert_count[NUM_CLASSES];
static long long bin_length_sum[NUM_CLASSES];

static void profile_print(void)
{
    fprintf(stderr, "\n[PROFILE] Bin 삽입 통계\n");
    fprintf(stderr, "class | 크기 범위              | 삽입 횟수 | 평균 리스트 길이\n");
    fprintf(stderr, "------+------------------------+-----------+----------------\n");

    size_t lo = 1, hi = 8;
    for (int i = 0; i < NUM_CLASSES; i++) {
        double avg = (bin_insert_count[i] > 0)
                     ? (double)bin_length_sum[i] / bin_insert_count[i]
                     : 0.0;
        if (i < NUM_CLASSES - 1)
            fprintf(stderr, "  %2d  | %6zu ~ %6zu B        | %9zu  | %8.2f\n",
                    i, lo, hi, bin_insert_count[i], avg);
        else
            fprintf(stderr, "  %2d  | %6zu B ~              | %9zu  | %8.2f\n",
                    i, lo, bin_insert_count[i], avg);
        lo = hi + 1;
        hi <<= 1;
    }
    fprintf(stderr, "\n→ 평균 길이가 길수록 정렬 비용 높음, 크기 범위가 넓을수록 정렬 효과 높음\n\n");
}

static void profile_record(int class)
{
    int len = 0;
    char *cur = seg_list[class];
    while (cur != NULL) {
        len++;
        cur = GET_PTR(NEXT_PTR(cur));
    }
    bin_insert_count[class]++;
    bin_length_sum[class] += len;
}
#endif

/* ================================================================
 * 내부 함수 전방 선언
 * ================================================================ */

static int   get_class(size_t size);
static void  insert_free(void *bp);
static void  remove_free(void *bp);
static void *coalesce(void *bp);
static void *extend_heap(size_t words);
static void *find_fit(size_t asize);
static void *place(void *bp, size_t asize);

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
 * insert_free - 가용 블록 bp를 해당 bin에 삽입
 *
 * [전략]
 *   소형 bin (class 0~6, ~512B): LIFO (맨 앞 삽입) → O(1), 속도 우선
 *   대형 bin (class 7~10, 513B~): 오름차순 정렬 삽입 → first-fit이 best-fit처럼 동작
 *
 * [임계 class 6/7의 근거 - 프로파일링 측정값]
 *
 *   class 1 (9~16B)  : 평균 리스트 길이 125 → 정렬 시 O(125) 비용,
 *                      하지만 범위가 8B로 좁아 어떤 블록도 fit → 정렬 효과 0
 *   class 2~6        : 평균 길이 1~3, 범위 좁음 → 정렬 효과 낮음, LIFO로 충분
 *   class 7~9        : 평균 길이 0.3~1.6, 범위 512B+ → 정렬 비용 ≈ O(1), 효과 있음
 *   class 10 (4097~) : 평균 길이 4.16, 범위 무한 → 정렬 없으면 과분할 심각
 *
 *   → class 7 이상부터 정렬하면 비용 대비 효과가 명확히 양수.
 */
static void insert_free(void *bp)
{
    int class = get_class(GET_SIZE(HDRP(bp)));
    size_t bp_size = GET_SIZE(HDRP(bp));

#ifdef PROFILE
    profile_record(class);  /* 삽입 전 현재 bin 길이 기록 */
#endif

    if (class <= 6) {
        /* 소형 bin: LIFO */
        char *head = seg_list[class];
        PUT_PTR(PREV_PTR(bp), NULL);
        PUT_PTR(NEXT_PTR(bp), head);
        if (head != NULL)
            PUT_PTR(PREV_PTR(head), bp);
        seg_list[class] = bp;
    } else {
        /* 대형 bin: 오름차순 정렬 삽입 */
        char *cur = seg_list[class];
        char *prev = NULL;
        while (cur != NULL && GET_SIZE(HDRP(cur)) < bp_size) {
            prev = cur;
            cur = GET_PTR(NEXT_PTR(cur));
        }
        PUT_PTR(NEXT_PTR(bp), cur);
        PUT_PTR(PREV_PTR(bp), prev);
        if (cur != NULL)
            PUT_PTR(PREV_PTR(cur), bp);
        if (prev != NULL)
            PUT_PTR(NEXT_PTR(prev), bp);
        else
            seg_list[class] = bp;
    }
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
 * SPLIT_THRESHOLD - 분할 방향 전환 기준 크기
 *
 * asize >= SPLIT_THRESHOLD 이면 블록 뒤쪽을 할당하고 앞쪽을 가용 블록으로 남김.
 * 이렇게 하면 작은 블록은 힙 앞쪽, 큰 블록은 힙 뒤쪽에 모여
 * binary 패턴의 단편화를 줄일 수 있음.
 */
#define SPLIT_THRESHOLD  96

/*
 * place - bp 블록에 asize 바이트 할당
 *
 * [분할 방향 전략]
 *   asize < SPLIT_THRESHOLD  → 앞쪽 할당, 뒤쪽 가용 (기존 방식)
 *   asize >= SPLIT_THRESHOLD → 뒤쪽 할당, 앞쪽 가용
 *   → 작은 블록/큰 블록이 힙에서 자연스럽게 분리되어 coalesce 가능성 증가
 *
 * 할당된 블록은 푸터 없음. 다음 블록의 prev_alloc 비트 업데이트 필요.
 */
static void *place(void *bp, size_t asize)
{
    size_t csize = GET_SIZE(HDRP(bp));
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(bp));

    remove_free(bp);

    if ((csize - asize) >= MIN_BLOCK_SIZE) {
        if (asize >= SPLIT_THRESHOLD) {
            /* 큰 요청: 뒤쪽 할당, 앞쪽을 가용 블록으로
             * rem(앞) → 가용, alloc_bp(뒤) → 할당
             * 반환값: alloc_bp (mm_malloc이 이 주소를 사용자에게 반환) */
            char *rem = bp;
            size_t rem_size = csize - asize;
            PUT(HDRP(rem), PACK(rem_size, prev_alloc, 0));
            PUT(FTRP(rem), PACK(rem_size, prev_alloc, 0));
            insert_free(rem);

            char *alloc_bp = NEXT_BLKP(rem);
            PUT(HDRP(alloc_bp), PACK(asize, 0, 1));
            /* 다음 블록의 prev_alloc 비트를 1로 */
            char *next = NEXT_BLKP(alloc_bp);
            size_t next_alloc = GET_ALLOC(HDRP(next));
            PUT(HDRP(next), PACK(GET_SIZE(HDRP(next)), 1, next_alloc));
            if (!next_alloc)
                PUT(FTRP(next), PACK(GET_SIZE(HDRP(next)), 1, 0));
            return alloc_bp;
        } else {
            /* 작은 요청: 앞쪽 할당, 뒤쪽을 가용 블록으로 */
            PUT(HDRP(bp), PACK(asize, prev_alloc, 1));

            char *next = NEXT_BLKP(bp);
            PUT(HDRP(next), PACK(csize - asize, 1, 0));
            PUT(FTRP(next), PACK(csize - asize, 1, 0));

            char *next2 = NEXT_BLKP(next);
            PUT(HDRP(next2), PACK(GET_SIZE(HDRP(next2)), 0, GET_ALLOC(HDRP(next2))));
            insert_free(next);
        }
    } else {
        /* 분할 없이 전체 할당 */
        PUT(HDRP(bp), PACK(csize, prev_alloc, 1));

        char *next = NEXT_BLKP(bp);
        size_t next_alloc = GET_ALLOC(HDRP(next));
        PUT(HDRP(next), PACK(GET_SIZE(HDRP(next)), 1, next_alloc));
        if (!next_alloc)
            PUT(FTRP(next), PACK(GET_SIZE(HDRP(next)), 1, 0));
    }
    return bp;
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
#ifdef PROFILE
    /* 이전 trace 통계 출력 후 초기화 */
    if (bin_insert_count[0] + bin_insert_count[NUM_CLASSES - 1] > 0)
        profile_print();
    memset(bin_insert_count, 0, sizeof(bin_insert_count));
    memset(bin_length_sum,   0, sizeof(bin_length_sum));
#endif

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
        return place(bp, asize);
    }

    extendsize = MAX(asize, CHUNKSIZE);
    if ((bp = extend_heap(extendsize / WSIZE)) == NULL)
        return NULL;
    return place(bp, asize);
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
 * - 축소 요청  : 현재 블록 그대로 반환
 * - 확장 요청  : 아래 순서로 제자리 확장 시도, 모두 실패 시 새 블록 할당
 *
 * [제자리 확장 우선순위]
 *   1. 다음 블록이 가용 → 흡수하여 확장
 *   2. 다음 블록이 에필로그(힙 끝) → 힙 확장 후 흡수
 *   3. 위 모두 실패 → malloc + memcpy + free
 *
 * [에필로그 확장 효과]
 *   realloc-bal.rep처럼 점진적으로 크기가 커지는 패턴에서
 *   힙 끝 블록을 바로 확장하여 이전 블록 자리에 단편화 방지.
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
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(ptr));

    /* 축소 또는 현재 크기로 충분한 경우: 그대로 반환 */
    if (asize <= cur_size)
        return ptr;

    char *next = NEXT_BLKP(ptr);
    size_t next_alloc = GET_ALLOC(HDRP(next));
    size_t next_size = GET_SIZE(HDRP(next));
    size_t combined = cur_size + next_size;

    /* Case 1: 다음 블록이 가용이고 합친 크기가 충분한 경우 → 제자리 확장 */
    if (!next_alloc && combined >= asize) {
        remove_free(next);

        if ((combined - asize) >= MIN_BLOCK_SIZE) {
            PUT(HDRP(ptr), PACK(asize, prev_alloc, 1));
            char *new_next = NEXT_BLKP(ptr);
            PUT(HDRP(new_next), PACK(combined - asize, 1, 0));
            PUT(FTRP(new_next), PACK(combined - asize, 1, 0));
            char *new_next2 = NEXT_BLKP(new_next);
            PUT(HDRP(new_next2), PACK(GET_SIZE(HDRP(new_next2)), 0, GET_ALLOC(HDRP(new_next2))));
            insert_free(new_next);
        } else {
            PUT(HDRP(ptr), PACK(combined, prev_alloc, 1));
            char *new_next = NEXT_BLKP(ptr);
            size_t na = GET_ALLOC(HDRP(new_next));
            PUT(HDRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, na));
            if (!na)
                PUT(FTRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, 0));
        }
        return ptr;
    }

    /* Case 2: 다음 블록이 에필로그(힙 끝) → 힙 확장 후 제자리 흡수 */
    if (next_size == 0) {
        size_t need = asize - cur_size;
        size_t extendsize = MAX(need, CHUNKSIZE);
        if (extend_heap(extendsize / WSIZE) == NULL)
            return NULL;

        /* extend_heap이 coalesce를 통해 가용 블록을 seg_list에 등록했으므로 제거 */
        char *new_free = NEXT_BLKP(ptr);  /* 확장된 가용 블록 */
        size_t new_free_size = GET_SIZE(HDRP(new_free));
        combined = cur_size + new_free_size;
        remove_free(new_free);

        if ((combined - asize) >= MIN_BLOCK_SIZE) {
            PUT(HDRP(ptr), PACK(asize, prev_alloc, 1));
            char *new_next = NEXT_BLKP(ptr);
            PUT(HDRP(new_next), PACK(combined - asize, 1, 0));
            PUT(FTRP(new_next), PACK(combined - asize, 1, 0));
            char *new_next2 = NEXT_BLKP(new_next);
            PUT(HDRP(new_next2), PACK(GET_SIZE(HDRP(new_next2)), 0, GET_ALLOC(HDRP(new_next2))));
            insert_free(new_next);
        } else {
            PUT(HDRP(ptr), PACK(combined, prev_alloc, 1));
            char *new_next = NEXT_BLKP(ptr);
            size_t na = GET_ALLOC(HDRP(new_next));
            PUT(HDRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, na));
            if (!na)
                PUT(FTRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, 0));
        }
        return ptr;
    }

    /* Case 3: 제자리 확장 불가 → 새 블록 할당 후 복사 */
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
