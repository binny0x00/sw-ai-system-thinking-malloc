/*
 * mm.c - Segregated free list + prev_alloc 비트 기반 malloc 구현
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    힙 전체 구조 (메모리 상)                    │
 * ├──────┬────────────┬────────────┬────────────┬──────────────┤
 * │패딩   │프롤로그헤더    │프롤로그푸터   │ 블록들...    │에필로그헤더     │
 * │(4B)  │(4B, size=8)│(4B, size=8)│            │(4B, size=0)  │
 * └──────┴────────────┴────────────┴────────────┴──────────────┘
 *   ↑ 정렬을 맞추기 위한 패딩
 *        ↑ 경계 조건용 sentinel (실제 데이터 아님)
 *
 * 프롤로그/에필로그가 필요한 이유:
 *   블록 병합(coalesce) 시 "이전/다음 블록이 힙 경계인지" 를 매번 검사하면 코드가 복잡해짐.
 *   프롤로그(항상 할당 상태)와 에필로그(항상 할당 상태, size=0)를 두면
 *   경계 검사 없이 일반 케이스만 처리해도 됨.
 *
 * ┌─────────────────────────────────────────────────────────────┐
 * │                    블록 구조                                  │
 * ├─────────────────────────────────────────────────────────────┤
 * │ 할당된 블록:  [헤더 4B] [페이로드] [패딩(선택)]                      │
 * │ 가용   블록:  [헤더 4B] [prev_offset 4B] [next_offset 4B]      │
 * │               [...비어있는 공간...] [푸터 4B]                   │
 * └─────────────────────────────────────────────────────────────┘
 *
 * 헤더/푸터 비트 구조 (4바이트 = 32비트):
 *   [ 블록크기(상위 29비트) | prev_alloc(bit1) | alloc(bit0) ]
 *   - alloc     : 1이면 현재 블록 할당 중, 0이면 가용
 *   - prev_alloc: 1이면 바로 앞 블록이 할당 중
 *                 → 앞 블록이 할당 중이면 앞 블록의 푸터가 없으므로
 *                   이 비트로 대신 앞 블록 상태를 파악
 *   - 크기는 항상 8의 배수이므로 하위 3비트는 항상 0 → 플래그 저장에 재활용
 *
 * [Segregated free list - 크기 구간별 bin 11개]
 *   bins[0] : 1    ~ 8B
 *   bins[1] : 9    ~ 16B
 *   bins[2] : 17   ~ 32B
 *   bins[3] : 33   ~ 64B
 *   bins[4] : 65   ~ 128B
 *   bins[5] : 129  ~ 256B
 *   bins[6] : 257  ~ 512B
 *   bins[7] : 513  ~ 1024B
 *   bins[8] : 1025 ~ 2048B
 *   bins[9] : 2049 ~ 4096B
 *   bins[10]: 4097B ~
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

#define WSIZE 4             /* 워드(헤더/푸터) 크기: 4바이트 */
#define DSIZE 8             /* 더블워드 크기: 8바이트 (정렬 기준) */
#define CHUNKSIZE (1 << 12) /* 힙 확장 단위: 4096바이트                                           \
                             * 한 번 확장할 때 너무 작으면 sbrk 호출이 잦아져 느려짐. \
                             * 너무 크면 메모리 낭비. 4KB는 OS 페이지 크기와 같아(대부분 페이지 크기와 동일) 적절. */
#define ALIGNMENT 8         /* 8바이트 정렬: x86-64에서 double/pointer의 정렬 요구사항 */
#define NUM_CLASSES 11      /* segregated bin 개수 */

#define MAX(x, y) ((x) > (y) ? (x) : (y))

/* ================================================================
 * 힙 오프셋 기반 포인터 (4바이트)
 *
 * [왜 8바이트 포인터 대신 4바이트 오프셋을 쓰나?]
 *
 * 64비트 환경에서 포인터는 8바이트.
 * 가용 블록에 prev/next 포인터를 저장하면 블록 최소 크기가
 *   header(4) + prev(8) + next(8) + footer(4) = 24바이트가 됨.
 *
 * 오프셋(= 블록주소 - 힙시작주소)을 4바이트 unsigned int로 저장하면:
 *   header(4) + prev_offset(4) + next_offset(4) + footer(4) = 16바이트
 * → 최소 블록 크기 24B → 16B로 줄어 소형 블록 단편화 감소.
 *
 * 힙 최대 크기 = 20MB < 4GB(2^32) 이므로 4바이트 오프셋으로 충분.
 * offset = 0은 NULL 표현 (실제 가용 블록의 최소 오프셋 = 16이므로 충돌 없음).
 * ================================================================ */
#define PTR_SIZE 4

/*
 * _get_ptr: 주소 p에 저장된 4바이트 오프셋을 실제 포인터로 변환
 *   offset == 0이면 NULL 반환 (리스트 끝 표시)
 *   아니면 mem_heap_lo() + offset = 실제 블록 주소
 */
static inline char *_get_ptr(void *p)
{
    unsigned int offset = *(unsigned int *)p;
    return offset == 0 ? NULL : (char *)mem_heap_lo() + offset;
}

/*
 * _put_ptr: 주소 p에 포인터 ptr을 4바이트 오프셋으로 변환하여 저장
 *   ptr == NULL이면 0 저장
 *   아니면 ptr - mem_heap_lo() = 오프셋 저장
 */
static inline void _put_ptr(void *p, void *ptr)
{
    *(unsigned int *)p = (ptr == NULL)
                             ? 0
                             : (unsigned int)((char *)ptr - (char *)mem_heap_lo());
}

#define GET_PTR(p) _get_ptr(p)
#define PUT_PTR(p, ptr) _put_ptr(p, (void *)(ptr))

/*
 * 가용 블록 최소 크기 = 16바이트
 *   header(4) + prev_offset(4) + next_offset(4) + footer(4)
 *
 * 이 크기보다 작으면 가용 상태가 되었을 때 prev/next 오프셋을 저장할 공간이 없음.
 * 따라서 할당 시 블록이 이 크기보다 작아지는 분할은 하지 않는다.
 */
#define MIN_BLOCK_SIZE (2 * WSIZE + 2 * PTR_SIZE) /* = 16 */

/* ================================================================
 * PACK / GET / PUT 매크로
 *
 * [PACK이 필요한 이유]
 * 헤더에 크기(29비트), prev_alloc(1비트), alloc(1비트)을 하나의 4바이트 값으로 저장.
 * 크기는 항상 8의 배수이므로 하위 3비트가 항상 0 → 플래그 저장에 재활용.
 *
 *   PACK(24, 1, 0) = 24 | (1<<1) | 0 = 0b...0011000 | 0b10 = 0b...0011010 = 26
 *                                                       ↑↑
 *                                               prev_alloc=1, alloc=0
 * ================================================================ */
#define PACK(size, prev_alloc, alloc) ((size) | ((prev_alloc) << 1) | (alloc))

/* 주소 p에서 4바이트 워드 읽기/쓰기 */
#define GET(p) (*(unsigned int *)(p))
#define PUT(p, val) (*(unsigned int *)(p) = (val))

/*
 * 헤더/푸터에서 필드 추출
 *
 * GET_SIZE  : 하위 3비트(플래그) 를 0으로 마스킹 → 크기만 추출
 *             ~0x7 = 0b...11111000 → AND하면 하위 3비트 제거
 * GET_ALLOC : bit0만 추출 → 현재 블록 할당 여부
 * GET_PREV_ALLOC: bit1을 1비트 오른쪽 시프트 → 이전 블록 할당 여부
 */
#define GET_SIZE(p) (GET(p) & ~0x7)
#define GET_ALLOC(p) (GET(p) & 0x1)
#define GET_PREV_ALLOC(p) ((GET(p) >> 1) & 0x1)

/* ================================================================
 * 블록 포인터(bp) 기반 주소 계산 매크로
 *
 * bp는 항상 블록의 "페이로드 시작 주소"를 가리킨다.
 * 헤더는 페이로드 바로 앞 4바이트에 위치.
 *
 *   메모리 배치:
 *   [헤더(4B)] [페이로드/prev_offset/next_offset ...] [푸터(4B)]
 *                ↑ bp가 여기를 가리킴
 *
 * HDRP(bp)  = bp - 4         : 헤더 주소
 * FTRP(bp)  = bp + size - 8  : 푸터 주소 (size에는 헤더+푸터 포함)
 *             (bp + size - DSIZE: 블록 끝에서 8바이트 앞 = 푸터 시작)
 * NEXT_BLKP : 현재 블록 끝 = bp + size → 다음 블록 페이로드 시작
 * PREV_BLKP : 바로 앞 블록의 푸터에서 크기를 읽어 역산
 *             (가용 블록만 푸터를 가지므로, 앞 블록이 가용일 때만 사용 가능)
 * ================================================================ */
#define HDRP(bp) ((char *)(bp) - WSIZE)
#define FTRP(bp) ((char *)(bp) + GET_SIZE(HDRP(bp)) - DSIZE)
#define NEXT_BLKP(bp) ((char *)(bp) + GET_SIZE(HDRP(bp)))
#define PREV_BLKP(bp) ((char *)(bp) - GET_SIZE((char *)(bp) - DSIZE))

/*
 * 가용 블록 내 연결 리스트 포인터 위치
 *
 * 가용 블록의 페이로드 영역에 prev/next 오프셋을 저장.
 *   [헤더] [prev_offset(4B)] [next_offset(4B)] [...] [푸터]
 *           ↑ bp             ↑ bp + 4
 *
 * PREV_PTR(bp): prev_offset이 저장된 주소 = bp (페이로드 시작)
 * NEXT_PTR(bp): next_offset이 저장된 주소 = bp + 4
 */
#define PREV_PTR(bp) ((char *)(bp))
#define NEXT_PTR(bp) ((char *)(bp) + PTR_SIZE)

/*
 * ALIGN(size): size를 8의 배수로 올림
 *
 * 왜 정렬이 필요한가?
 *   CPU가 8바이트 경계에서 메모리를 읽을 때 가장 효율적.
 *   double, 포인터 등이 8바이트 정렬을 요구하므로,
 *   malloc이 반환하는 주소는 반드시 8의 배수여야 한다.
 *
 * 동작 원리:
 *   (size + 7) & ~7
 *   예) size=13: (13+7)=20, 20 & ~7 = 20 & 0b...11111000 = 16
 *   예) size=16: (16+7)=23, 23 & ~7 = 16 (이미 정렬된 경우 그대로)
 */
#define ALIGN(size) (((size) + (ALIGNMENT - 1)) & ~(ALIGNMENT - 1))

/* ================================================================
 * 전역 변수
 * ================================================================ */

/*
 * seg_list[i]: i번째 크기 구간의 가용 블록 연결 리스트 헤드 포인터
 *
 * segregated free list를 쓰는 이유:
 *   단일 연결 리스트(implicit/explicit)는 모든 가용 블록을 처음부터 탐색 → O(n).
 *   크기별로 bin을 나누면 요청 크기에 맞는 bin부터 탐색 → 평균 O(1)에 가까워짐.
 *
 * 각 bin은 이중 연결 리스트:
 *   NULL ← [블록A] ↔ [블록B] ↔ [블록C] → NULL
 *   (prev/next 오프셋으로 연결)
 */
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
static size_t bin_insert_count[NUM_CLASSES];
static long long bin_length_sum[NUM_CLASSES];

static void profile_print(void)
{
    fprintf(stderr, "\n[PROFILE] Bin 삽입 통계\n");
    fprintf(stderr, "class | 크기 범위              | 삽입 횟수 | 평균 리스트 길이\n");
    fprintf(stderr, "------+------------------------+-----------+----------------\n");

    size_t lo = 1, hi = 8;
    for (int i = 0; i < NUM_CLASSES; i++)
    {
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
    while (cur != NULL)
    {
        len++;
        cur = GET_PTR(NEXT_PTR(cur));
    }
    bin_insert_count[class]++;
    bin_length_sum[class] += len;
}
#endif

/* ================================================================
 * 내부 함수 전방 선언
 *
 * C는 함수를 위에서 아래로 읽으므로, 호출보다 먼저 선언되어 있어야 함.
 * 구현보다 먼저 선언만 해두면 컴파일러가 타입을 미리 알 수 있음.
 * ================================================================ */

static int get_class(size_t size);
static void insert_free(void *bp);
static void remove_free(void *bp);
static void *coalesce(void *bp);
static void *extend_heap(size_t words);
static void *find_fit(size_t asize);
static void *place(void *bp, size_t asize);

/* ================================================================
 * 내부 함수 구현
 * ================================================================ */

/*
 * get_class - size가 속하는 segregated bin 인덱스 반환
 *
 * 크기 구간이 2배씩 커지는 이유:
 *   프로그램의 메모리 요청 크기는 지수적으로 분포하는 경향이 있음.
 *   (소형 요청이 매우 많고, 대형 요청은 드묾)
 *   2배 단위로 나누면 bin 개수를 적게 유지하면서도 크기를 잘 구분 가능.
 *
 * 동작:
 *   s를 8부터 시작해서 2배씩 키우며 size > s인 동안 class를 증가.
 *   s <<= 1 은 s = s * 2 와 동일 (비트 시프트로 빠르게 계산).
 *
 *   예) size=100: 100>8, 100>16, 100>32, 100>64, 100<=128 → class=4 (65~128B)
 */
static int get_class(size_t size)
{
    int class = 0;
    size_t s = 8;
    while (class < NUM_CLASSES - 1 && size > s)
    {
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
 *
 * [LIFO 삽입 그림]
 *   새 블록 → [새블록] → [기존헤드] → ...
 *   seg_list[class] = 새블록
 *
 * [정렬 삽입 그림]
 *   [100B] → [200B] → [500B] → NULL
 *   새 블록(300B) 삽입 후:
 *   [100B] → [200B] → [300B] → [500B] → NULL
 *                        ↑ find_fit이 이 블록부터 찾아줌 → 낭비 최소화
 */
static void insert_free(void *bp)
{
    int class = get_class(GET_SIZE(HDRP(bp)));
    size_t bp_size = GET_SIZE(HDRP(bp));

#ifdef PROFILE
    profile_record(class); /* 삽입 전 현재 bin 길이 기록 */
#endif

    if (class <= 6)
    {
        /* 소형 bin: LIFO - 맨 앞에 삽입 */
        char *head = seg_list[class];
        PUT_PTR(PREV_PTR(bp), NULL); /* bp의 prev = NULL (리스트 맨 앞) */
        PUT_PTR(NEXT_PTR(bp), head); /* bp의 next = 기존 헤드 */
        if (head != NULL)
            PUT_PTR(PREV_PTR(head), bp); /* 기존 헤드의 prev = bp */
        seg_list[class] = bp;            /* 헤드를 bp로 교체 */
    }
    else
    {
        /* 대형 bin: 오름차순 정렬 삽입 */
        char *cur = seg_list[class];
        char *prev = NULL;
        /* bp_size보다 작은 블록들을 건너뜀 → bp가 들어갈 위치(cur 앞) 탐색 */
        while (cur != NULL && GET_SIZE(HDRP(cur)) < bp_size)
        {
            prev = cur;
            cur = GET_PTR(NEXT_PTR(cur));
        }
        /* prev → bp → cur 순서로 삽입 */
        PUT_PTR(NEXT_PTR(bp), cur);
        PUT_PTR(PREV_PTR(bp), prev);
        if (cur != NULL)
            PUT_PTR(PREV_PTR(cur), bp);
        if (prev != NULL)
            PUT_PTR(NEXT_PTR(prev), bp);
        else
            seg_list[class] = bp; /* 리스트에서 가장 작은 블록이면 헤드로 */
    }
}

/*
 * remove_free - 가용 리스트에서 블록 bp 제거
 *
 * 언제 호출되나?
 *   1. 가용 블록을 할당할 때 (place 함수 내부)
 *   2. 인접 가용 블록과 병합할 때 (coalesce 함수 내부)
 *   3. realloc에서 다음 가용 블록을 흡수할 때
 *
 * [그림] prev → bp → next 에서 bp 제거 → prev → next
 *
 *   prev가 있으면: prev의 next를 bp의 next로 교체
 *   prev가 없으면: bp가 헤드였으므로 bin 헤드를 bp의 next로 교체
 *   next가 있으면: next의 prev를 bp의 prev로 교체
 */
static void remove_free(void *bp)
{
    int class = get_class(GET_SIZE(HDRP(bp)));
    char *prev = GET_PTR(PREV_PTR(bp));
    char *next = GET_PTR(NEXT_PTR(bp));

    if (prev != NULL)
        PUT_PTR(NEXT_PTR(prev), next);
    else
        seg_list[class] = next; /* bp가 헤드였으면 헤드를 next로 교체 */

    if (next != NULL)
        PUT_PTR(PREV_PTR(next), prev);
}

/*
 * coalesce - 방금 가용 상태가 된 블록 bp를 인접 가용 블록과 병합
 *
 * 왜 병합이 필요한가?
 *   free를 여러 번 하면 인접한 작은 가용 블록들이 생김.
 *   이들을 합치지 않으면 큰 요청을 충족할 수 없어 단편화가 심해짐.
 *   예) 16B + 16B + 16B가 인접해 있는데 40B 요청이 들어오면?
 *       병합 없이는 40B 블록이 없어 힙을 새로 확장해야 함 → 낭비
 *
 * prev_alloc 비트를 쓰는 이유:
 *   일반적으로 이전 블록의 상태를 알려면 이전 블록의 헤더를 읽어야 하는데,
 *   이전 블록의 헤더 위치는 이전 블록의 푸터를 읽어야 알 수 있음.
 *   그런데 할당된 블록은 푸터를 생략(공간 절약)하므로 푸터를 읽을 수 없음.
 *   → 현재 블록 헤더의 prev_alloc 비트에 이전 블록 상태를 기록해 해결.
 *
 * [4가지 케이스]
 *   Case 1: 앞 할당, 뒤 할당 → 병합 없음, bp만 가용으로 표시
 *   Case 2: 앞 할당, 뒤 가용 → bp + 뒤 블록 병합
 *   Case 3: 앞 가용, 뒤 할당 → 앞 블록 + bp 병합
 *   Case 4: 앞 가용, 뒤 가용 → 앞 + bp + 뒤 전부 병합
 *
 * 병합 후 반드시 해야 할 일:
 *   병합된 블록의 다음 블록 헤더에서 prev_alloc 비트를 0으로 갱신.
 *   (이제 앞 블록이 가용이므로)
 */
static void *coalesce(void *bp)
{
    /* 현재 블록의 헤더에서 앞/뒤 블록 상태 읽기 */
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(bp));
    size_t next_alloc = GET_ALLOC(HDRP(NEXT_BLKP(bp)));
    size_t size = GET_SIZE(HDRP(bp));
    char *next_blk;

    if (prev_alloc && next_alloc)
    {
        /* Case 1: 양쪽 모두 할당 → 병합 없이 가용 블록으로만 표시 */
        PUT(HDRP(bp), PACK(size, 1, 0));
        PUT(FTRP(bp), PACK(size, 1, 0));
        next_blk = NEXT_BLKP(bp);
        /* 다음 블록에게 "앞이 이제 가용이야" 알림 */
        PUT(HDRP(next_blk), PACK(GET_SIZE(HDRP(next_blk)), 0, GET_ALLOC(HDRP(next_blk))));
    }
    else if (prev_alloc && !next_alloc)
    {
        /* Case 2: 뒤 블록과 병합
         * 뒤 블록을 리스트에서 제거하고, 두 블록을 합쳐서 하나의 큰 블록으로 */
        remove_free(NEXT_BLKP(bp));
        size += GET_SIZE(HDRP(NEXT_BLKP(bp)));
        PUT(HDRP(bp), PACK(size, 1, 0));
        PUT(FTRP(bp), PACK(size, 1, 0)); /* FTRP는 새 size 기준으로 계산됨 */
        next_blk = NEXT_BLKP(bp);
        PUT(HDRP(next_blk), PACK(GET_SIZE(HDRP(next_blk)), 0, GET_ALLOC(HDRP(next_blk))));
    }
    else if (!prev_alloc && next_alloc)
    {
        /* Case 3: 앞 블록과 병합
         * 앞 블록을 리스트에서 제거, bp를 앞 블록 시작점으로 이동 */
        size_t pp_alloc = GET_PREV_ALLOC(HDRP(PREV_BLKP(bp))); /* 앞앞 블록 상태 보존 */
        remove_free(PREV_BLKP(bp));
        size += GET_SIZE(HDRP(PREV_BLKP(bp)));
        bp = PREV_BLKP(bp); /* bp를 앞 블록 시작으로 이동 */
        PUT(HDRP(bp), PACK(size, pp_alloc, 0));
        PUT(FTRP(bp), PACK(size, pp_alloc, 0));
        next_blk = NEXT_BLKP(bp);
        PUT(HDRP(next_blk), PACK(GET_SIZE(HDRP(next_blk)), 0, GET_ALLOC(HDRP(next_blk))));
    }
    else
    {
        /* Case 4: 양쪽 모두 가용 → 세 블록 전부 병합
         * 앞/뒤 블록 모두 리스트에서 제거 후 하나로 합침 */
        size_t pp_alloc = GET_PREV_ALLOC(HDRP(PREV_BLKP(bp)));
        remove_free(NEXT_BLKP(bp));
        remove_free(PREV_BLKP(bp));
        size += GET_SIZE(HDRP(PREV_BLKP(bp))) +
                GET_SIZE(HDRP(NEXT_BLKP(bp)));
        bp = PREV_BLKP(bp);
        PUT(HDRP(bp), PACK(size, pp_alloc, 0));
        PUT(FTRP(bp), PACK(size, pp_alloc, 0));
        next_blk = NEXT_BLKP(bp);
        PUT(HDRP(next_blk), PACK(GET_SIZE(HDRP(next_blk)), 0, GET_ALLOC(HDRP(next_blk))));
    }

    /* 병합된 블록을 가용 리스트에 삽입 */
    insert_free(bp);
    return bp;
}

/*
 * extend_heap - 힙을 words 워드만큼 확장하고 새 가용 블록 반환
 *
 * 언제 호출되나?
 *   1. mm_init: 프로그램 시작 시 초기 힙 공간 확보
 *   2. mm_malloc: 기존 가용 블록으로 요청을 충족할 수 없을 때
 *   3. mm_realloc: 힙 끝 블록을 확장해야 할 때
 *
 * [힙 확장 전]  ... [마지막 블록] [에필로그 헤더(size=0, alloc=1)]
 *                                  ↑ mem_brk (힙 끝)
 *
 * [힙 확장 후]  ... [마지막 블록] [새 가용 블록 헤더] [...] [새 가용 블록 푸터] [새 에필로그]
 *
 * words를 짝수로 올림하는 이유:
 *   정렬(8바이트) 유지. 홀수 × 4 = 4의 배수지만 8의 배수가 아닐 수 있음.
 *   짝수 × 4 = 항상 8의 배수.
 *
 * 확장 후 coalesce를 호출하는 이유:
 *   힙 끝의 마지막 블록이 가용 상태였다면 새 블록과 합칠 수 있음.
 *   이를 합치지 않으면 작은 가용 블록들이 힙 끝에 흩어짐.
 */
static void *extend_heap(size_t words)
{
    char *bp;
    size_t size;

    /* 짝수 워드로 올림 (8바이트 정렬 유지) */
    size = (words % 2) ? (words + 1) * WSIZE : words * WSIZE;
    if ((long)(bp = mem_sbrk(size)) == -1)
        return NULL;

    /*
     * mem_sbrk는 확장 전 brk 포인터(= 에필로그가 있던 자리)를 반환.
     * 즉, bp는 기존 에필로그 헤더가 있던 위치의 페이로드 시작을 가리킴.
     * HDRP(bp) = bp - 4 = 기존 에필로그 헤더 위치 → 여기서 prev_alloc 읽기.
     */
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(bp));

    PUT(HDRP(bp), PACK(size, prev_alloc, 0)); /* 새 가용 블록 헤더 */
    PUT(FTRP(bp), PACK(size, prev_alloc, 0)); /* 새 가용 블록 푸터 */
    PUT(HDRP(NEXT_BLKP(bp)), PACK(0, 0, 1));  /* 새 에필로그: size=0, alloc=1 */

    /* 앞 블록이 가용이었다면 병합 */
    return coalesce(bp);
}

/*
 * find_fit - asize 이상인 가용 블록 탐색
 *
 * [segregated first-fit 전략]
 *   1. asize가 속하는 bin부터 탐색 시작 (너무 작은 bin은 건너뜀)
 *   2. 해당 bin에서 first-fit: 첫 번째 맞는 블록 반환
 *   3. 없으면 더 큰 bin으로 이동
 *
 * 대형 bin은 오름차순 정렬되어 있으므로 first-fit = best-fit 효과.
 * 소형 bin은 LIFO지만 범위가 좁아 어떤 블록이든 fit → 첫 블록 반환.
 *
 * 예) asize=100B → get_class(100)=4 (65~128B bin)부터 탐색
 *     bin4에 맞는 블록 없으면 → bin5(129~256B) 탐색 → ...
 */
static void *find_fit(size_t asize)
{
    for (int i = get_class(asize); i < NUM_CLASSES; i++)
    {
        char *bp = seg_list[i];
        while (bp != NULL)
        {
            if (GET_SIZE(HDRP(bp)) >= asize)
                return bp;
            bp = GET_PTR(NEXT_PTR(bp));
        }
    }
    return NULL; /* 모든 bin에서 못 찾으면 NULL → 힙 확장 필요 */
}

/*
 * SPLIT_THRESHOLD - 분할 방향 전환 기준 크기 (96바이트)
 *
 * [왜 분할 방향이 중요한가?]
 *
 * 기본 방식(앞쪽 할당): 큰 블록의 앞부분에 할당하고 나머지를 가용으로 남김.
 *   [alloc | free]
 *
 * 문제: 작은 요청(alloc)과 큰 요청(alloc)이 번갈아오면
 *   힙이 [작음|큰가용][작음|큰가용][작음|큰가용]... 패턴이 됨.
 *   큰 가용 블록들이 중간중간 작은 할당 블록에 막혀 서로 합쳐지지 못함 → 단편화.
 *
 * 해결: 큰 요청은 블록 뒤쪽에 할당.
 *   작은 요청: [alloc | free] → 작은 블록이 앞쪽에 모임
 *   큰 요청:   [free | alloc] → 큰 블록이 뒤쪽에 모임
 *   → 작은 블록들끼리, 큰 블록들끼리 인접하게 되어 coalesce 가능성 증가.
 */
#define SPLIT_THRESHOLD 96

/*
 * place - find_fit으로 찾은 블록 bp에 asize 바이트 할당
 *
 * [3가지 경우]
 *
 * 1. 분할 가능 + 큰 요청 (asize >= SPLIT_THRESHOLD):
 *    [    csize    ] → [rem(가용) | alloc_bp(할당)]
 *    앞을 가용으로 남기고 뒤에 할당. alloc_bp 반환.
 *
 * 2. 분할 가능 + 작은 요청 (asize < SPLIT_THRESHOLD):
 *    [    csize    ] → [bp(할당) | next(가용)]
 *    앞에 할당하고 뒤를 가용으로 남김. bp 반환.
 *
 * 3. 분할 불가 (남는 공간 < MIN_BLOCK_SIZE):
 *    [    csize    ] → [bp(할당, csize 전체)]
 *    그냥 전체를 할당. 약간의 내부 단편화 발생하지만 허용.
 *
 * [반환값이 중요한 이유]
 *   큰 요청 시 alloc_bp = bp + rem_size. bp와 다른 주소.
 *   mm_malloc이 place의 반환값을 써야 올바른 주소를 사용자에게 줄 수 있음.
 *
 * [할당 후 반드시 해야 할 일]
 *   할당된 블록은 푸터 없음. 하지만 다음 블록이 현재 블록 상태를 알아야 하므로
 *   다음 블록 헤더의 prev_alloc 비트를 1로 갱신.
 */
static void *place(void *bp, size_t asize)
{
    size_t csize = GET_SIZE(HDRP(bp));
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(bp));

    /* 할당할 블록을 가용 리스트에서 먼저 제거 */
    remove_free(bp);

    if ((csize - asize) >= MIN_BLOCK_SIZE)
    {
        if (asize >= SPLIT_THRESHOLD)
        {
            /* 큰 요청: 뒤쪽 할당, 앞쪽(rem)을 가용 블록으로 남김 */
            char *rem = bp;
            size_t rem_size = csize - asize;
            PUT(HDRP(rem), PACK(rem_size, prev_alloc, 0)); /* rem 헤더: 가용 */
            PUT(FTRP(rem), PACK(rem_size, prev_alloc, 0)); /* rem 푸터 */
            insert_free(rem);

            char *alloc_bp = NEXT_BLKP(rem);        /* rem 바로 뒤 = 실제 할당 위치 */
            PUT(HDRP(alloc_bp), PACK(asize, 0, 1)); /* alloc_bp 헤더: 할당, prev_alloc=0(rem이 가용) */
            /* alloc_bp의 다음 블록에 "앞이 할당됨" 알림 */
            char *next = NEXT_BLKP(alloc_bp);
            size_t next_alloc = GET_ALLOC(HDRP(next));
            PUT(HDRP(next), PACK(GET_SIZE(HDRP(next)), 1, next_alloc));
            if (!next_alloc)
                PUT(FTRP(next), PACK(GET_SIZE(HDRP(next)), 1, 0));
            return alloc_bp; /* ← 실제 할당된 위치 반환 */
        }
        else
        {
            /* 작은 요청: 앞쪽 할당, 뒤쪽을 가용 블록으로 남김 */
            PUT(HDRP(bp), PACK(asize, prev_alloc, 1)); /* bp 헤더: 할당 */

            char *next = NEXT_BLKP(bp);
            PUT(HDRP(next), PACK(csize - asize, 1, 0)); /* 나머지 블록 헤더: 가용, prev_alloc=1 */
            PUT(FTRP(next), PACK(csize - asize, 1, 0)); /* 나머지 블록 푸터 */

            /* 나머지 블록의 다음 블록에 "앞이 이제 가용" 알림 */
            char *next2 = NEXT_BLKP(next);
            PUT(HDRP(next2), PACK(GET_SIZE(HDRP(next2)), 0, GET_ALLOC(HDRP(next2))));
            insert_free(next);
        }
    }
    else
    {
        /* 분할 불가: 블록 전체를 할당 (약간의 내부 단편화 허용) */
        PUT(HDRP(bp), PACK(csize, prev_alloc, 1));

        /* 다음 블록에 "앞이 할당됨" 알림 */
        char *next = NEXT_BLKP(bp);
        size_t next_alloc = GET_ALLOC(HDRP(next));
        PUT(HDRP(next), PACK(GET_SIZE(HDRP(next)), 1, next_alloc));
        if (!next_alloc)
            PUT(FTRP(next), PACK(GET_SIZE(HDRP(next)), 1, 0));
    }
    return bp;
}

/* ================================================================
 * 공개 함수 구현 (mm.h에 선언된 인터페이스)
 * ================================================================ */

/*
 * mm_init - malloc 패키지 초기화
 *
 * mdriver가 각 trace 시작 전에 호출.
 * 힙을 완전히 초기화하고 초기 가용 블록을 만든다.
 *
 * [초기 힙 구조 (16바이트)]
 *   주소:  0    4         8         12
 *          [패딩][프롤로그H][프롤로그F][에필로그H]
 *
 * 패딩(4B): mem_sbrk가 반환하는 주소를 8바이트 정렬로 맞추기 위함.
 *   (패딩 4B + 프롤로그H 4B = 8B → 이후 페이로드가 8B 정렬)
 *
 * 프롤로그(size=8, alloc=1): 항상 할당 상태인 sentinel.
 *   coalesce가 "힙 시작을 넘어서 앞으로 가는" 상황을 방지.
 *
 * 에필로그(size=0, alloc=1): 항상 할당 상태인 sentinel.
 *   coalesce가 "힙 끝을 넘어서 뒤로 가는" 상황을 방지.
 *   extend_heap 시 이 자리가 새 블록의 헤더 위치가 됨.
 */
int mm_init(void)
{
    // 성능 디버깅 용도
#ifdef PROFILE
    /* 이전 trace 통계 출력 후 초기화 */
    if (bin_insert_count[0] + bin_insert_count[NUM_CLASSES - 1] > 0)
        profile_print();
    // 각 bin에서 삽입이 얼마나 자주 일어났는지
    memset(bin_insert_count, 0, sizeof(bin_insert_count));
    // 삽입 시 리스트 길이가 평균 얼마나 길었는지
    memset(bin_length_sum, 0, sizeof(bin_length_sum));
#endif

    /* 모든 bin 헤드를 NULL로 초기화 */
    for (int i = 0; i < NUM_CLASSES; i++)
        seg_list[i] = NULL;

    char *heap_start;
    if ((heap_start = mem_sbrk(4 * WSIZE)) == (void *)-1)
        return -1;

    PUT(heap_start, 0);                               /* 정렬 패딩 */
    PUT(heap_start + (1 * WSIZE), PACK(DSIZE, 1, 1)); /* 프롤로그 헤더: size=8, prev=1, alloc=1 */
    PUT(heap_start + (2 * WSIZE), PACK(DSIZE, 1, 1)); /* 프롤로그 푸터 */
    PUT(heap_start + (3 * WSIZE), PACK(0, 1, 1));     /* 에필로그 헤더: size=0, alloc=1 */

    /* 초기 가용 블록 생성 (CHUNKSIZE = 4096B) */
    if (extend_heap(CHUNKSIZE / WSIZE) == NULL)
        return -1;
    return 0;
}

/*
 * mm_malloc - size 바이트 페이로드를 담을 블록 할당, 페이로드 주소 반환
 *
 * [할당 크기(asize) 계산]
 *   사용자가 요청한 size에 헤더(4B)를 더하고 8바이트 정렬로 올림.
 *   단, 나중에 이 블록이 해제되면 가용 블록이 되어 prev/next 오프셋(각 4B)을
 *   저장해야 하므로 MIN_BLOCK_SIZE(16B) 이상을 보장.
 *
 *   예) size=1  → ALIGN(1+4)=8  < 16 → asize=16
 *       size=13 → ALIGN(13+4)=24 > 16 → asize=24
 *
 * [탐색 → 없으면 확장]
 *   find_fit으로 기존 가용 블록 탐색.
 *   없으면 MAX(asize, CHUNKSIZE) 만큼 힙 확장.
 *   (CHUNKSIZE보다 큰 요청이면 그만큼, 아니면 최소 4096B 확장)
 */
void *mm_malloc(size_t size)
{
    size_t asize;
    size_t extendsize;
    char *bp;

    if (size == 0)
        return NULL;

    asize = MAX(ALIGN(size + WSIZE), MIN_BLOCK_SIZE);

    if ((bp = find_fit(asize)) != NULL)
    {
        return place(bp, asize); /* place의 반환값을 그대로 반환 (분할 방향 때문에 bp와 다를 수 있음) */
    }

    extendsize = MAX(asize, CHUNKSIZE);
    if ((bp = extend_heap(extendsize / WSIZE)) == NULL)
        return NULL;
    return place(bp, asize);
}

/*
 * mm_free - ptr이 가리키는 블록 해제
 *
 * [동작]
 *   1. 헤더에서 크기와 prev_alloc을 읽음
 *   2. 헤더를 "가용" 상태로 덮어씀 (alloc 비트 0)
 *   3. 푸터를 새로 씀 (할당 중에는 푸터가 없었으므로)
 *   4. coalesce로 인접 가용 블록과 병합 + 리스트에 삽입
 *
 * prev_alloc은 왜 유지하나?
 *   현재 블록 해제 후에도 앞 블록의 상태는 변하지 않음.
 *   coalesce에서 Case 1/2 vs Case 3/4를 구분하기 위해 유지.
 */
void mm_free(void *ptr)
{
    size_t size = GET_SIZE(HDRP(ptr));
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(ptr));

    PUT(HDRP(ptr), PACK(size, prev_alloc, 0)); /* 헤더: alloc=0으로 변경 */
    PUT(FTRP(ptr), PACK(size, prev_alloc, 0)); /* 푸터 생성 (병합 시 필요) */

    coalesce(ptr); /* 인접 가용 블록과 병합 후 리스트 삽입 */
}

/*
 * mm_realloc - ptr 블록의 크기를 size 바이트로 조정
 *
 * [특수 케이스]
 *   ptr == NULL → malloc(size)와 동일
 *   size == 0   → free(ptr)와 동일, NULL 반환
 *
 * [핵심 아이디어: 제자리 확장으로 memcpy 최소화]
 *   단순 구현: malloc(size) → memcpy → free(ptr)
 *   문제: 데이터 복사 비용 크고, 이전 자리에 단편화 발생.
 *
 *   개선: 현재 블록 자리에서 확장 가능한지 먼저 시도.
 *
 * [확장 우선순위]
 *   Case 1. 다음 블록이 가용이고 합산 크기가 충분
 *     → 다음 블록을 흡수하여 제자리 확장. memcpy 불필요.
 *     → 남는 공간이 MIN_BLOCK_SIZE 이상이면 뒤쪽을 새 가용 블록으로 분할.
 *
 *   Case 2. 다음 블록이 에필로그 (힙 끝)
 *     → 힙을 확장하면 새 가용 블록이 현재 블록 바로 뒤에 생김.
 *     → 그 가용 블록을 흡수하여 제자리 확장. memcpy 불필요.
 *     → realloc-bal.rep처럼 점점 크기가 커지는 패턴에서 큰 효과.
 *
 *   Case 3. 위 모두 불가
 *     → 새 위치에 malloc → memcpy → free(ptr)
 */
void *mm_realloc(void *ptr, size_t size)
{
    if (ptr == NULL)
        return mm_malloc(size);

    if (size == 0)
    {
        mm_free(ptr);
        return NULL;
    }

    size_t asize = MAX(ALIGN(size + WSIZE), MIN_BLOCK_SIZE);
    size_t cur_size = GET_SIZE(HDRP(ptr));
    size_t prev_alloc = GET_PREV_ALLOC(HDRP(ptr));

    /* 현재 블록이 이미 충분히 크면 그대로 반환 (축소는 하지 않음) */
    if (asize <= cur_size)
        return ptr;

    char *next = NEXT_BLKP(ptr);
    size_t next_alloc = GET_ALLOC(HDRP(next));
    size_t next_size = GET_SIZE(HDRP(next));
    size_t combined = cur_size + next_size;

    /* Case 1: 다음 블록이 가용이고 합산 크기가 충분 */
    if (!next_alloc && combined >= asize)
    {
        remove_free(next); /* 다음 블록을 가용 리스트에서 제거 */

        if ((combined - asize) >= MIN_BLOCK_SIZE)
        {
            /* 남는 공간을 새 가용 블록으로 분할 */
            PUT(HDRP(ptr), PACK(asize, prev_alloc, 1));
            char *new_next = NEXT_BLKP(ptr);
            PUT(HDRP(new_next), PACK(combined - asize, 1, 0));
            PUT(FTRP(new_next), PACK(combined - asize, 1, 0));
            char *new_next2 = NEXT_BLKP(new_next);
            PUT(HDRP(new_next2), PACK(GET_SIZE(HDRP(new_next2)), 0, GET_ALLOC(HDRP(new_next2))));
            insert_free(new_next);
        }
        else
        {
            /* 남는 공간이 너무 작으면 전체를 할당 블록으로 */
            PUT(HDRP(ptr), PACK(combined, prev_alloc, 1));
            char *new_next = NEXT_BLKP(ptr);
            size_t na = GET_ALLOC(HDRP(new_next));
            PUT(HDRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, na));
            if (!na)
                PUT(FTRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, 0));
        }
        return ptr;
    }

    /* Case 2: 다음 블록이 에필로그(size=0) → 힙 확장 후 제자리 흡수 */
    if (next_size == 0)
    {
        size_t need = asize - cur_size;           /* 추가로 필요한 바이트 */
        size_t extendsize = MAX(need, CHUNKSIZE); /* 최소 CHUNKSIZE만큼 확장 */
        if (extend_heap(extendsize / WSIZE) == NULL)
            return NULL;

        /*
         * extend_heap → coalesce → insert_free 순으로 새 가용 블록이 리스트에 등록됨.
         * 이 블록을 현재 ptr에 흡수할 것이므로 리스트에서 먼저 제거.
         */
        char *new_free = NEXT_BLKP(ptr); /* 확장으로 생긴 가용 블록 */
        size_t new_free_size = GET_SIZE(HDRP(new_free));
        combined = cur_size + new_free_size;
        remove_free(new_free);

        if ((combined - asize) >= MIN_BLOCK_SIZE)
        {
            PUT(HDRP(ptr), PACK(asize, prev_alloc, 1));
            char *new_next = NEXT_BLKP(ptr);
            PUT(HDRP(new_next), PACK(combined - asize, 1, 0));
            PUT(FTRP(new_next), PACK(combined - asize, 1, 0));
            char *new_next2 = NEXT_BLKP(new_next);
            PUT(HDRP(new_next2), PACK(GET_SIZE(HDRP(new_next2)), 0, GET_ALLOC(HDRP(new_next2))));
            insert_free(new_next);
        }
        else
        {
            PUT(HDRP(ptr), PACK(combined, prev_alloc, 1));
            char *new_next = NEXT_BLKP(ptr);
            size_t na = GET_ALLOC(HDRP(new_next));
            PUT(HDRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, na));
            if (!na)
                PUT(FTRP(new_next), PACK(GET_SIZE(HDRP(new_next)), 1, 0));
        }
        return ptr;
    }

    /* Case 3: 제자리 확장 불가 → 새 위치에 할당 후 데이터 복사 */
    void *newptr = mm_malloc(size);
    if (newptr == NULL)
        return NULL;

    /* 복사할 바이트 수: 현재 블록의 페이로드 크기와 새 요청 크기 중 작은 값 */
    size_t copySize = cur_size - WSIZE; /* 헤더 4B 제외한 페이로드 크기 */
    if (size < copySize)
        copySize = size;
    memcpy(newptr, ptr, copySize);

    mm_free(ptr);
    return newptr;
}
