#
# Students' Makefile for the Malloc Lab
# "어떻게 컴파일할 것인가"를 정의하는 빌드 자동화 스크립트
#
TEAM = bovik
VERSION = 1
HANDINDIR = /afs/cs.cmu.edu/academic/class/15213-f01/malloclab/handin

CC = gcc
CFLAGS = -Wall -O2 -g

OBJS = mdriver.o mm.o memlib.o fsecs.o fcyc.o clock.o ftimer.o

mdriver: $(OBJS)
	$(CC) $(CFLAGS) -o mdriver $(OBJS)

# 프로파일링 빌드: bin별 삽입 횟수와 평균 리스트 길이를 stderr에 출력
# 사용법: make profile && ./mdriver -V 2>profile.txt && cat profile.txt
profile:
	$(MAKE) CFLAGS="$(CFLAGS) -DPROFILE"

mdriver.o: mdriver.c fsecs.h fcyc.h clock.h memlib.h config.h mm.h
memlib.o: memlib.c memlib.h
mm.o: mm.c mm.h memlib.h
fsecs.o: fsecs.c fsecs.h config.h
fcyc.o: fcyc.c fcyc.h
ftimer.o: ftimer.c ftimer.h config.h
clock.o: clock.c clock.h

handin:
	cp mm.c $(HANDINDIR)/$(TEAM)-$(VERSION)-mm.c

clean:
	rm -f *~ *.o mdriver


