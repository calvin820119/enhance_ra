all:
	gcc -o a.o main.c lcgrand.c -lm
clean:
	rm *.o
run:
	./a.o
