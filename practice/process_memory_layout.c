#include <stdio.h>

int global_init = 10; // .data
int global_uninit;    // .bss

int main()
{
    int local = 5;              // stack
    static int static_var = 20; // .data
    printf("%d\n", global_init);
}

/*
// 전처리만 수행 (-E)
gcc -E process_memory_layout.c > cpp_output.c

// 컴파일->어셈블리까지만 (-S)
gcc -S process_memory_layout.c

// 링크 없이 오브젝트 파일 생성 (-c)
gcc -c process_memory_layout.c

// .o 파일 내용 읽기 위한 명령어
// 내부 코드 보기(-d:disassemble)
objdump -d process_memory_layout.o
// 파일 구조 보기(-h:header)
objdump -h process_memory_layout.o

// 링커 연결된 실행 파일 생성
gcc process_memory_layout.c -o program

// .text 확인
objdump -d program
//.data 확인
objdump -s -j .data program
//.bss 확인
readelf -S program
//.rodata 확인
objdump -s -j .rodata program
*/