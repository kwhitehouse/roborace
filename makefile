OBJS = Parser.o algs.o main.o 
CC = g++
DEBUG = -g
CFLAGS = -Wall -c $(DEBUG)
LFLAGS = -Wall $(DEBUG)

letz_race : $(OBJS)
	$(CC) $(LFLAGS) $(OBJS) -o letz_race
